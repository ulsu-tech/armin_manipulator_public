#ifndef CUSTOM_TYPES_H
#define CUSTOM_TYPES_H

#pragma pack(push, 1)
typedef struct data {
    uint8_t tag;
    int16_t sensor_x;
    int16_t sensor_y;
    int16_t sensor_z;    // optosensor (3*2=6)
    uint8_t button;      // Button ball (1)
    uint16_t tenzor[3];  // 3*2=        (6)
            int32_t enc1;  // position of shaft 1
            int32_t enc2;  // position of shaft 2
            int32_t enc3;  // position of shaft 3
    uint16_t crc;
} PositionalData; 

#define BUTTONS_USER_PRESENCE (1<<0)
#define BUTTONS_ASYNC_ACTION (1<<1)
#define BUTTON_OPEN_RQ       (1<<2)
#define BUTTON_CLOSE_RQ      (1<<3)
#define ROTATION_ONLY_BTN    (1<<4)
#define IGNORE_ORIENTATION_BTN    (1<<5)

#define IS_USER_PRESENT(x) (x.button & BUTTONS_USER_PRESENCE)
#define IS_BUTTON_OPEN(x) (x.button & BUTTON_OPEN_RQ)
#define IS_BUTTON_CLOSE(x) (x.button & BUTTON_CLOSE_RQ)
#define IS_ONLY_ROTATION_FORCED(x) (x.button & ROTATION_ONLY_BTN)
#define IGNORE_ORIENTATION_CLICKED(x) (x.button & IGNORE_ORIENTATION_BTN)

typedef struct tool_position {
    uint8_t tag;
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t crc;
} ToolPositionInform;


#pragma pack(pop)

#endif
