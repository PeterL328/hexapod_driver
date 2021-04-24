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

#endif //JOINT_SERVO_MAPPING_H