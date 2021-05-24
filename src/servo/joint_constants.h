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

// Head
constexpr uint8_t HEAD_UPDOWN_CH     = 0;
constexpr uint8_t HEAD_LEFTRIGHT_CH  = 1;

// Joint offsets. Unit in degrees
// Left front leg
constexpr float LF_COXA_OFFSET       = 8.5f;
constexpr float LF_FEMUR_OFFSET      = 0.0f;
constexpr float LF_TIBIA_OFFSET      = -10.0f;

// Left middle leg
constexpr float LM_COXA_OFFSET       = 14.5f;
constexpr float LM_FEMUR_OFFSET      = 14.0f;
constexpr float LM_TIBIA_OFFSET      = -13.5f;

// Left back leg
constexpr float LB_COXA_OFFSET       = 7.5f;
constexpr float LB_FEMUR_OFFSET      = -3.0f;
constexpr float LB_TIBIA_OFFSET      = -7.0f;

// Right front leg
constexpr float RF_COXA_OFFSET       = 7.0f;
constexpr float RF_FEMUR_OFFSET      = 2.0f;
constexpr float RF_TIBIA_OFFSET      = 12.5f;

// Right middle leg
constexpr float RM_COXA_OFFSET       = 8.0f;
constexpr float RM_FEMUR_OFFSET      = -12.0f;
constexpr float RM_TIBIA_OFFSET      = -3.0f;

// Right back leg
constexpr float RB_COXA_OFFSET       = 9.0f;
constexpr float RB_FEMUR_OFFSET      = 7.0f;
constexpr float RB_TIBIA_OFFSET      = -8.0f;

// Head
constexpr float H_UPDOWN_OFFSET      = 7.0f;
constexpr float H_LEFTRIGHT_OFFSET   = 0.0f;

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

// Head
constexpr bool H_UPDOWN_DIR       = true;
constexpr bool H_LEFTRIGHT_DIR    = false;

#endif //JOINT_SERVO_MAPPING_H