#ifndef __OLED_API_H
#define __OLED_API_H


#define OLED_USE_MENU         1         //置1使用多级菜单OLED，置0使用江科大版本OLED


#if OLED_USE_MENU
#include "./oled_ui/oled_ui.h"
#include "./oled_ui/oled_ui_menudata.h"
#else 
#include "./oled_jiangkeda/oled.h"
#endif



#endif
