#ifndef __OLED_UI_MENUDATA_H
#define __OLED_UI_MENUDATA_H
// 检测是否是C++编译器
#ifdef __cplusplus
extern "C" {
#endif
#include "oled_ui.h"

//进行前置声明
#if 0
extern MenuItem MainMenuItems[],SettingsMenuItems[],AboutThisDeviceMenuItems[],
AboutOLED_UIMenuItems[],MoreMenuItems[],Font8MenuItems[] ,Font12MenuItems[] ,
Font16MenuItems[] ,Font20MenuItems[],LongMenuItems[],SpringMenuItems[],LongListMenuItems[],SmallAreaMenuItems[];
extern MenuPage MainMenuPage,SettingsMenuPage,AboutThisDeviceMenuPage,
AboutOLED_UIMenuPage,MoreMenuPage,Font8MenuPage,Font12MenuPage,Font16MenuPage
,Font20MenuPage,LongMenuPage,SpringMenuPage,LongListMenuPage,SmallAreaMenuPage;
#endif
/*上面为示例，下面依照示例声明自己的变量*/
/* ★2026.10.6 合并说明：
   - MainMenuItems/MainMenuPage 用的是队友 2026.10.2 版（比赛流程参数：出发/圆盘机/识别/阶梯/立柱/回家，全部可在上电 KEY0 菜单里调）。
   - PIDParamMenuItems/OtherParamMenuItems 与两个菜单页是本工程原有，仍保留（挂在 MainMenuPage 末尾）。
   - 退出菜单用队友的 exit&save / exit&cancel 两个标志（原 oled_ui_exit 已废弃）。 */
extern MenuItem MainMenuItems[], PIDParamMenuItems[], OtherParamMenuItems[];
extern MenuPage MainMenuPage, PIDParamMenuPage, OtherParamMenuPage;
extern bool oled_ui_exit_save, oled_ui_exit_cancel;


#ifdef __cplusplus
}  // extern "C"
#endif

#endif
