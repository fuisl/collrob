/* Include the controller definition */
#include "footbot_flocking.h"

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

/* std */
#include <limits>
#include <algorithm>

namespace {
/* Small epsilon to avoid division-by-zero / angle issues */
constexpr Real EPS = 1e-6f;

/* Clamp helper (C++11) */
inline Real Clamp01(Real x) {
   if(x < 0.0f) return 0.0f;
   if(x > 1.0f) return 1.0f;
   return x;
}
} // namespace

/****************************************/
/****************************************/

void CFootBotFlocking::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotFlocking::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotFlocking::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   /* Guard */
   if(f_distance < EPS) f_distance = EPS;
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

CFootBotFlocking::CFootBotFlocking() :
   m_pcWheels(nullptr),
   m_pcLight(nullptr),
   m_pcLEDs(nullptr),
   m_pcCamera(nullptr),
   m_pcProximity(nullptr),
   /* avoidance params (kept small by design) */
   m_fAvoidStart(0.16f),
   m_fAvoidFull(0.28f),
   m_fAvoidGain(0.25f),
   m_fLastMaxProx(0.0f),
   m_fProxMag(0.0f) {}

/****************************************/
/****************************************/

void CFootBotFlocking::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator          >("differential_steering");
   m_pcLight     = GetSensor  <CCI_FootBotLightSensor                    >("footbot_light");
   m_pcLEDs      = GetActuator<CCI_LEDsActuator                          >("leds");
   m_pcCamera    = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor                >("footbot_proximity");

   /*
    * Parse the config file
    */
   try {
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      m_sFlockingParams.Init(GetNode(t_node, "flocking"));

      /* Optional avoidance section */
      if(NodeExists(t_node, "avoidance")) {
         TConfigurationNode& tAvoid = GetNode(t_node, "avoidance");
         GetNodeAttributeOrDefault(tAvoid, "start", m_fAvoidStart, m_fAvoidStart);
         GetNodeAttributeOrDefault(tAvoid, "full",  m_fAvoidFull,  m_fAvoidFull);
         GetNodeAttributeOrDefault(tAvoid, "gain",  m_fAvoidGain,  m_fAvoidGain);
      }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   Reset();
}

/****************************************/
/****************************************/

void CFootBotFlocking::Reset() {
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);
   /* Reset last readings */
   m_fLastMaxProx = 0.0f;
   m_fProxMag = 0.0f;
}

/****************************************/
/****************************************/
/* Goal (pick ONE depending on your XML) */

/*
 * If your goal is a REAL <light medium="lights">,
 * then the original VectorToLight() is correct and robust.
 *
 * If your goal is a <light medium="leds"> (i.e., LED medium),
 * the light sensor won't see it, so you MUST use the camera target blob.
 *
 * This implementation tries light first (original behavior),
 * then falls back to yellow blob (so your current LED-medium map still works).
 */
CVector2 CFootBotFlocking::GoalVector() {
   CVector2 vLight = VectorToLight();
   if(vLight.Length() > EPS) return vLight;
   return VectorToTargetBlob(); /* works if goal appears as YELLOW blob */
}

/****************************************/
/****************************************/

CVector2 CFootBotFlocking::VectorToLight() {
   const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
   CVector2 cAccum;

   for(size_t i = 0; i < tReadings.size(); ++i) {
      cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
   }

   if(cAccum.Length() > EPS) {
      cAccum.Normalize();
      cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed; /* original scaling */
   }
   return cAccum;
}

/****************************************/
/****************************************/

CVector2 CFootBotFlocking::VectorToTargetBlob() {
   const auto& sReadings = m_pcCamera->GetReadings();
   if(sReadings.BlobList.empty()) return CVector2();

   Real     fBestDist  = std::numeric_limits<Real>::max();
   CRadians cBestAngle = CRadians::ZERO;
   bool     bFound     = false;

   for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
      const auto* psBlob = sReadings.BlobList[i];
      if(psBlob->Color == CColor::YELLOW) {
         if(psBlob->Distance < fBestDist) {
            fBestDist  = psBlob->Distance;
            cBestAngle = psBlob->Angle;
            bFound = true;
         }
      }
   }

   if(!bFound) return CVector2();

   /* Keep goal pull "original-like": modest, not dominating flock */
   CVector2 cGoal(1.0f, cBestAngle);
   cGoal.Normalize();
   cGoal *= 0.25f * m_sWheelTurningParams.MaxSpeed;
   return cGoal;
}

/****************************************/
/****************************************/
/* Flocking (unchanged from original) */

CVector2 CFootBotFlocking::FlockingVector() {
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings =
      m_pcCamera->GetReadings();

   if(sReadings.BlobList.empty()) {
      return CVector2();
   }

   CVector2 cAccum;
   Real     fLJ;
   size_t   unBlobsSeen = 0;

   for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
      if(sReadings.BlobList[i]->Color == CColor::RED &&
         sReadings.BlobList[i]->Distance < m_sFlockingParams.TargetDistance * 1.80f) {

         fLJ = m_sFlockingParams.GeneralizedLennardJones(sReadings.BlobList[i]->Distance);

         cAccum += CVector2(fLJ, sReadings.BlobList[i]->Angle);
         ++unBlobsSeen;
      }
   }

   if(unBlobsSeen == 0) return CVector2();

   cAccum /= unBlobsSeen;

   if(cAccum.Length() > m_sWheelTurningParams.MaxSpeed) {
      cAccum.Normalize();
      cAccum *= m_sWheelTurningParams.MaxSpeed;
   }

   return cAccum;
}

/****************************************/
/****************************************/
/* Avoidance (small and only near-contact) */

Real CFootBotFlocking::AvoidanceWeight(Real fProxMag) const {
   if(fProxMag <= m_fAvoidStart) return 0.0f;
   if(fProxMag >= m_fAvoidFull)  return 1.0f;
   return Clamp01((fProxMag - m_fAvoidStart) / (m_fAvoidFull - m_fAvoidStart));
}

CVector2 CFootBotFlocking::ObstacleAvoidanceVector() {
   const CCI_FootBotProximitySensor::TReadings& tReads = m_pcProximity->GetReadings();
   if(tReads.empty()) {
      m_fProxMag = 0.0f;
      return CVector2();
   }

   /* Same structure as diffusion: sum then average */
   CVector2 cAccum;
   Real fMax = 0.0f;

   for(size_t i = 0; i < tReads.size(); ++i) {
      cAccum += CVector2(tReads[i].Value, tReads[i].Angle);
      fMax = Max(fMax, tReads[i].Value);
   }

   cAccum /= tReads.size();

   m_fProxMag      = cAccum.Length(); /* scalar "blockedness" */
   m_fLastMaxProx  = fMax;

   if(m_fProxMag <= EPS) return CVector2();

   /* Repel away */
   cAccum.Normalize();
   cAccum = -cAccum;

   /* Keep avoidance modest (you control via XML gain too) */
   cAccum *= (m_fAvoidGain * m_sWheelTurningParams.MaxSpeed);

   return cAccum;
}

/****************************************/
/****************************************/
/* ControlStep: original-style sum + tiny avoidance */

void CFootBotFlocking::ControlStep() {
   /* Original components */
   const CVector2 vGoal  = GoalVector();       /* light first, else yellow blob */
   const CVector2 vFlock = FlockingVector();

   /* Tiny local avoidance */
   const CVector2 vAvoid = ObstacleAvoidanceVector();
   const Real     wAvoid = AvoidanceWeight(m_fProxMag);

   /*
    * Keep original "feel":
    * - goal + flock always active
    * - avoidance only nudges near-contact
    */
   const Real kAvoidNudge = 0.15f; /* deliberately small */

   CVector2 vHeading = vGoal + vFlock + (kAvoidNudge * wAvoid) * vAvoid;

   /* If the goal is invisible and no neighbors are visible, drift forward */
   if(vHeading.Length() <= EPS) {
      vHeading = CVector2(0.2f * m_sWheelTurningParams.MaxSpeed, CRadians::ZERO);
   }

   SetWheelSpeedsFromVector(vHeading);
}

/****************************************/
/****************************************/
/* Wheel steering (unchanged from original) */

void CFootBotFlocking::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   Real     fHeadingLength = c_heading.Length();

   Real fBaseAngularWheelSpeed =
      Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }

   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         Real fSpeedFactor =
            (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
            m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }

   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }

   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotFlocking, "footbot_flocking_controller")
