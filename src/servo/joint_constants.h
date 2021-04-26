#ifndef JOINT_SERVO_MAPPING_H
#define JOINT_SERVO_MAPPING_H

// Contains joint to servo channel mapping
// Left front leg
constexpr uint8_t LF_COXA_CH         = 16;
constexpr uint8_t LF_FEMUR_CH        = 17;
constexpr uint8_t LF_TIBIA_CH        = 18;

// Left middle leg
constexpr uint8_t LM_COXA_CH         = 19;
constexpr uint8_t LM_FEMUR_CH        = 20;
constexpr uint8_t LM_TIBIA_CH        = 21;

// Left back leg
constexpr uint8_t LB_COXA_CH         = 22;
constexpr uint8_t LB_FEMUR_CH        = 23;
constexpr uint8_t LB_TIBIA_CH        = 27;

// Right front leg
constexpr uint8_t RF_COXA_CH         = 15;
constexpr uint8_t RF_FEMUR_CH        = 14;
constexpr uint8_t RF_TIBIA_CH        = 13;

// Right middle leg
constexpr uint8_t RM_COXA_CH         = 12;
constexpr uint8_t RM_FEMUR_CH        = 11;
constexpr uint8_t RM_TIBIA_CH        = 10;

// Right back leg
constexpr uint8_t RB_COXA_CH         = 9;
constexpr uint8_t RB_FEMUR_CH        = 8;
constexpr uint8_t RB_TIBIA_CH        = 31;

// Direction of the rotation for the servo
// Left front leg
constexpr bool LF_COXA_DIR        = true;
constexpr bool LF_FEMUR_DIR       = true;
constexpr bool LF_TIBIA_DIR       = true;

// Left middle leg
constexpr bool LM_COXA_DIR        = true;
constexpr bool LM_FEMUR_DIR       = true;
constexpr bool LM_TIBIA_DIR       = true;

// Left back leg
constexpr bool LB_COXA_DIR        = true;
constexpr bool LB_FEMUR_DIR       = true;
constexpr bool LB_TIBIA_DIR       = true;

// Right front leg
constexpr bool RF_COXA_DIR        = false;
constexpr bool RF_FEMUR_DIR       = false;
constexpr bool RF_TIBIA_DIR       = false;

// Right middle leg
constexpr bool RM_COXA_DIR        = false;
constexpr bool RM_FEMUR_DIR       = false;
constexpr bool RM_TIBIA_DIR       = false;

// Right back leg
constexpr bool RB_COXA_DIR        = false;
constexpr bool RB_FEMUR_DIR       = false;
constexpr bool RB_TIBIA_DIR       = false;

#endif //JOINT_SERVO_MAPPING_H