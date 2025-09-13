#include "cpu0_task.h"
#include "ssd1362.h"
#include "SPIFFS.h"
#include "robot.h"


#define SHOW_MODE_DEFAULT 0
#define SHOW_MODE_USER_ASSIGN 1

#define SLEEP_COUNT_DOWN 10

void maneuver_to_expression(void);
void delayed_sleep_mode(void);
bool refresh_maneuver_state(void);
void show_assign_expression(String expression_file,bool* exit);
void default_show_mode(void);
bool show_mode_check(void);
bool ten_msec_tick_cpu0(void);
void show_user_assign_expression(void);

// typedef bool (*exit)();

uint16_t sleep_time;


String show_file_name;
File file;
int maneuver_state = 0;
int frames_time = 500;
uint8_t last_frames_width;
uint8_t show_mode = SHOW_MODE_DEFAULT;
uint8_t *frames_data;
struct{
  uint8_t frames_num;
  uint8_t width;
  uint8_t height;
  uint8_t gray_level;
  uint8_t b[6];

}file_header;

uint16_t frames_num;
uint16_t width;
uint16_t height;
uint8_t *code;
uint16_t frames_size;
uint8_t x_coord;

void cpu0_task(void *ptParam) {

  frames_data = (uint8_t *)malloc(5128);
  SPIFFS.begin(true);
  OLED_Init();
  while (1) {
    show_mode_check();
    switch (show_mode) {
      case SHOW_MODE_DEFAULT:
      {
        default_show_mode();
      }
      break;
      case SHOW_MODE_USER_ASSIGN:
      {
        show_user_assign_expression();
      }
      break;
    }
  }
}

enum {
  ADVANCE = 0x0001,         //前
  RETREAT = 0x0002,         //后
  TERN_LEFT = 0x0004,       //左
  TERN_RIGHT = 0x0008,      //右
  RISE = 0x0010,            //上升
  DECLINE = 0x0020,         //下降
  GO = 0x0040,

  DANCE1 = 0x0100,
  DANCE2 = 0x0200,
  DANCE3 = 0x0400,
  DANCE4 = 0x0800,
} ManeuverStateTypDef;

//When the time is up, prepare to exit. The reason for using -1 is that 0 indicates that the timer is invalid.
bool exit_user_expression(void)
{
  if(rp.show_expression_time == -1){
    return true;
  }
  return false;
}
bool exit_default_show_mode(void)
{
  if(refresh_maneuver_state() == true){
    return true;
  }
  if(rp.show_expression_time > -1){
    return true;
  }
  return false;
}
void default_show_mode(void) {
  
  maneuver_to_expression();

  if (!SPIFFS.exists(show_file_name))   //file does not exist
  {
    OLED_Fill(0, 0, OLED_W, OLED_H, 0x00);
    String show_str = "No file " + show_file_name;
    OLED_ShowString(0, 0, (char*)show_str.c_str() , 16, 0);
    delay(1);
    return;
  }

  file = SPIFFS.open(show_file_name);  //Open the file and parse the file header
  file.read((uint8_t*)&file_header, sizeof(file_header));
  frames_num = file_header.frames_num;
  width = file_header.width;
  height = file_header.height;
  x_coord = (160 - width) / 2;

  /*
  If the image sizes of the two files are different, then the screen needs to be cleared once.
  */
  if (last_frames_width != width) {     
    OLED_Fill(0, 0, OLED_W, OLED_H, 0x00);
  }
  last_frames_width = width; 
  uint8_t i=0;
  for(i=0;i<frames_num;i++){ //
    if(file_header.gray_level == 2){      //homochromy
      frames_size = width * height / 8;
      file.read(frames_data, frames_size);
      OLED_DrawSingleBMP(x_coord, 0, width, height, frames_data,10, 0);
    }else if(file_header.gray_level == 16){//16 levels of gray
      frames_size = width * height / 2;
      file.read(frames_data, frames_size);
      OLED_DrawBMP(x_coord, 0, width, height, frames_data, 0);
    }
    // OLED_ShowNum(0, 0, i, 2, 16, 0);
    delay(frames_time);
    if(exit_default_show_mode() == true){
      file.close();
      return;
    }
  }
  file.close();
}

void show_user_assign_expression(void) {

  show_file_name = rp.show_expression;

  if (!SPIFFS.exists(show_file_name))   // 
  {
    OLED_Fill(0, 0, OLED_W, OLED_H, 0x00);
    OLED_ShowString(0, 0, "No file ...", 16, 0);
    delay(1);
    return;
  }

  file = SPIFFS.open(show_file_name);  // 
  file.read((uint8_t*)&file_header, sizeof(file_header));
  frames_num = file_header.frames_num;
  width = file_header.width;
  height = file_header.height;
  x_coord = (160 - width) / 2;
  if (last_frames_width != width) {     // 
    OLED_Fill(0, 0, OLED_W, OLED_H, 0x00);
  }
  last_frames_width = width;    // 
  uint8_t i=0;
  for(i=0;i<frames_num;i++){ // 
    if(file_header.gray_level == 2){      // 
      frames_size = width * height / 8;
      file.read(frames_data, frames_size);
      OLED_DrawSingleBMP(x_coord, 0, width, height, frames_data,10, 0);
    }else if(file_header.gray_level == 16){// 
      frames_size = width * height / 2;
      file.read(frames_data, frames_size);
      OLED_DrawBMP(x_coord, 0, width, height, frames_data, 0);
    }
    // OLED_ShowNum(0, 0, i, 2, 16, 0);
    delay(frames_time);
    if(exit_user_expression() == true){
      file.close();
      return;
    } 
  }
  file.close();
}

bool show_mode_check(void) {
 
  switch (show_mode) {
    case SHOW_MODE_DEFAULT:
      {
        if (rp.show_expression_time > -1)  //If the time is valid, it indicates that the specified expression needs to be displayed.
        {
          show_mode = SHOW_MODE_USER_ASSIGN;
          frames_time = 50;
          return true;
        }
      }
      break;
    case SHOW_MODE_USER_ASSIGN:
      {
        if (rp.show_expression_time == -1 )  //-1 indicates that the time has arrived.
        {
          show_mode = SHOW_MODE_DEFAULT;
          return true;
        }
      }
      break;
  }
  return false;
}
bool refresh_maneuver_state(void) {
  static int last_maneuver_state;

  maneuver_state = 0;
  if (wrobot.go > 0) {
    sleep_time = SLEEP_COUNT_DOWN;
    maneuver_state |= GO;
  }

  if (wrobot.joyy > 30)
    maneuver_state |= ADVANCE;
  else if (wrobot.joyy < -30)
    maneuver_state |= RETREAT;

  if (wrobot.joyx > 30)
    maneuver_state |= TERN_LEFT;
  else if (wrobot.joyx < -30)
    maneuver_state |= TERN_RIGHT;

  if (wrobot.height > 70)
    maneuver_state |= RISE;
  else if (wrobot.height < 38)
    maneuver_state |= DECLINE;

  if (last_maneuver_state != maneuver_state) {
    last_maneuver_state = maneuver_state;
    return true;
  }
  return false;
}
void maneuver_to_expression(void) {

  refresh_maneuver_state();
  frames_time = 50;

  if (!(maneuver_state & GO)) {
    delayed_sleep_mode();
  } else if (wrobot.uncontrolable == 1) {
    show_file_name = ("/Bored.bin"); 
  } else if (maneuver_state & RISE) {
    show_file_name = ("/Rise.bin");
  } else if (maneuver_state & ADVANCE) {
    show_file_name = ("/Advance.bin");
  } else if (maneuver_state & RETREAT) {
    show_file_name = ("/Retreat.bin");
  } else if (maneuver_state & TERN_LEFT) {
    show_file_name = ("/Left.bin");
  } else if (maneuver_state & TERN_RIGHT) {
    show_file_name = ("/Right.bin");
  } else if (maneuver_state & DANCE1) {
    show_file_name = ("/Pained.bin");
  } else if (maneuver_state & DANCE2) {
    show_file_name = ("/Pained.bin");
  } else if (maneuver_state & DANCE3) {
    show_file_name = ("/Pained.bin");
  } else if (maneuver_state & DANCE4) {
    show_file_name = ("/Pained.bin");
  } else {
    if (maneuver_state & DECLINE) {
      show_file_name = ("/Standby-Look.bin");
    } else {
      show_file_name = ("/Bored.bin");
    }
  }
}

void delayed_sleep_mode() {
  if (sleep_time > SLEEP_COUNT_DOWN / 2) {
    show_file_name = ("/Bored.bin");   //
    sleep_time--;
  } else if (sleep_time > 0) {
    show_file_name = ("/Relaxed.bin");      //
    sleep_time--;
  } else {
    show_file_name = ("/z.bin");
    frames_time = 500;
  }
}









