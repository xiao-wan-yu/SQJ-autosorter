#include "oled_ui_menudata.h"
#include "oled_ui.h"

/*此文件用于存放菜单数据。实际上菜单数据可以存放在任何地方，存放于此处是为了规范与代码模块化*/

/*上面为示例，下面依照示例声明自己要用到的变量*/
#include "./../storage.h"
bool oled_ui_exit_save = false;
bool oled_ui_exit_cancel = false;
bool sensor_data_show = false;



#define SPEED 10





//主LOGO移动的结构体
OLED_ChangePoint LogoMove;
//主LOGO文字移动的结构体
OLED_ChangePoint LogoTextMove;
//welcome文字移动的结构体
OLED_ChangePoint WelcomeTextMove;

extern OLED_ChangePoint OLED_UI_PageStartPoint ;


//主菜单的辅助显示函数
void MainAuxFunc(void){
	//不显示
	LogoMove.TargetPoint.X = -200;
	LogoMove.TargetPoint.Y = 0;
	LogoMove.CurrentPoint.X = -200;
	LogoMove.CurrentPoint.Y = 0;

	LogoTextMove.TargetPoint.X = 129;
	LogoTextMove.TargetPoint.Y = 0;
	LogoTextMove.CurrentPoint.X = 129;
	LogoTextMove.CurrentPoint.Y = 0;
	
	WelcomeTextMove.TargetPoint.X = 128;
	WelcomeTextMove.TargetPoint.Y = 0;
	WelcomeTextMove.CurrentPoint.X = 128;
	WelcomeTextMove.CurrentPoint.Y = 0;
}

















/*上面为示例，下面依照示例创建自己的窗口变量和函数 关于窗口的结构体*/
MenuWindow R_ChuFa_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "R_ChuFa_x",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.R_ChuFa_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -150,									//最小值
	.Prob_MaxData = 150, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
void Show_R_ChuFa_x_Window(void){
	OLED_UI_CreateWindow(&R_ChuFa_x_Window);
}
MenuWindow R_ChuFa_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "R_ChuFa_y",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.R_ChuFa_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = 0,									//最小值
	.Prob_MaxData = 550, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
void Show_R_ChuFa_y_Window(void){
	OLED_UI_CreateWindow(&R_ChuFa_y_Window);
}
MenuWindow B_ChuFa_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "B_ChuFa_x",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.B_ChuFa_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -150,									//最小值
	.Prob_MaxData = 150, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
void Show_B_ChuFa_x_Window(void){
	OLED_UI_CreateWindow(&B_ChuFa_x_Window);
}
MenuWindow B_ChuFa_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "B_ChuFa_y",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.B_ChuFa_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = 0,									//最小值
	.Prob_MaxData = 550, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
void Show_B_ChuFa_y_Window(void){
	OLED_UI_CreateWindow(&B_ChuFa_y_Window);
}



MenuWindow R_YuanPan_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "R_YuanPan_y",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.R_YuanPan_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -20,									//最小值
	.Prob_MaxData = 20, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
void Show_R_YuanPan_y_Window(void){
	OLED_UI_CreateWindow(&R_YuanPan_y_Window);
}
MenuWindow B_YuanPan_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "B_YuanPan_y",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.B_YuanPan_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -20,									//最小值
	.Prob_MaxData = 20, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
void Show_B_YuanPan_y_Window(void){
	OLED_UI_CreateWindow(&B_YuanPan_y_Window);
}




MenuWindow C_GetTarget_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_GetTarget_y",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_GetTarget_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = 0,									//最小值
	.Prob_MaxData = 200, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
void Show_C_GetTarget_y_Window(void){
	OLED_UI_CreateWindow(&C_GetTarget_y_Window);
}
MenuWindow C_GetTarget_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_GetTarget_x",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_GetTarget_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -100,									//最小值
	.Prob_MaxData = 0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
void Show_C_GetTarget_x_Window(void){
	OLED_UI_CreateWindow(&C_GetTarget_x_Window);
}




/*阶梯相关*/
MenuWindow C_JieTi_Left_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_JieTi_Left_x",				//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_JieTi_Left_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -60,									//最小值
	.Prob_MaxData = 60, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_JieTi_Left_x_Window(void){
	OLED_UI_CreateWindow(&C_JieTi_Left_x_Window);
}
MenuWindow C_Jieti_FwdTarget_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_Jieti_FwdTarget_y",		//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_Jieti_FwdTarget_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = 0,									//最小值
	.Prob_MaxData = 200, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_Jieti_FwdTarget_y_Window(void){
	OLED_UI_CreateWindow(&C_Jieti_FwdTarget_y_Window);
}
MenuWindow C_JitTi_Step_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_JitTi_Step_x",				//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_JitTi_Step_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -60,									//最小值
	.Prob_MaxData = 60, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_JitTi_Step_x_Window(void){
	OLED_UI_CreateWindow(&C_JitTi_Step_x_Window);
}
MenuWindow C_Jieti_Cross_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_Jieti_Cross_x",				//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_Jieti_Cross_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -60,									//最小值
	.Prob_MaxData = 60, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_Jieti_Cross_x_Window(void){
	OLED_UI_CreateWindow(&C_Jieti_Cross_x_Window);
}
MenuWindow C_JieTi_JiaoZhun_x1_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_JieTi_JiaoZhun_x1",		//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_JieTi_JiaoZhun_x1,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -30,									//最小值
	.Prob_MaxData = 30, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_JieTi_JiaoZhun_x1_Window(void){
	OLED_UI_CreateWindow(&C_JieTi_JiaoZhun_x1_Window);
}
MenuWindow C_JieTi_JiaoZhun_x2_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_JieTi_JiaoZhun_x2",		//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_JieTi_JiaoZhun_x2,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -30,									//最小值
	.Prob_MaxData = 30, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_JieTi_JiaoZhun_x2_Window(void){
	OLED_UI_CreateWindow(&C_JieTi_JiaoZhun_x2_Window);
}
MenuWindow C_JieTi_JiaoZhun_y1_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_JieTi_JiaoZhun_y1",		//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_JieTi_JiaoZhun_y1,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -30,									//最小值
	.Prob_MaxData = 30, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_JieTi_JiaoZhun_y1_Window(void){
	OLED_UI_CreateWindow(&C_JieTi_JiaoZhun_y1_Window);
}
MenuWindow C_JieTi_JiaoZhun_y2_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_JieTi_JiaoZhun_y2",		//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_JieTi_JiaoZhun_y2,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -30,									//最小值
	.Prob_MaxData = 30, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_JieTi_JiaoZhun_y2_Window(void){
	OLED_UI_CreateWindow(&C_JieTi_JiaoZhun_y2_Window);
}




/*立桩相关*/
MenuWindow R_LiZhuang_Delay_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "R_LiZhuang_Delay_x",		//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.R_LiZhuang_Delay_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = 0,									//最小值
	.Prob_MaxData = 1000, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_R_LiZhuang_Delay_x_Window(void){
	OLED_UI_CreateWindow(&R_LiZhuang_Delay_x_Window);
}
MenuWindow B_LiZhuang_Delay_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "B_LiZhuang_Delay_x",		//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.B_LiZhuang_Delay_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = 0,									//最小值
	.Prob_MaxData = 1000, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_B_LiZhuang_Delay_x_Window(void){
	OLED_UI_CreateWindow(&B_LiZhuang_Delay_x_Window);
}
MenuWindow C_LiZhuang_Target_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_LiZhuang_Target_y",		//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_LiZhuang_Target_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = 0,									//最小值
	.Prob_MaxData = 300, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_LiZhuang_Target_y_Window(void){
	OLED_UI_CreateWindow(&C_LiZhuang_Target_y_Window);
}
MenuWindow C_LiZhuang_Fwd_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_LiZhuang_Fwd_y",			//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_LiZhuang_Fwd_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -60,									//最小值
	.Prob_MaxData = 60, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_LiZhuang_Fwd_y_Window(void){
	OLED_UI_CreateWindow(&C_LiZhuang_Fwd_y_Window);
}
MenuWindow R_LiZhuang_Back_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "R_LiZhuang_Back_y",			//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.R_LiZhuang_Back_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -60,									//最小值
	.Prob_MaxData = 60, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_R_LiZhuang_Back_y_Window(void){
	OLED_UI_CreateWindow(&R_LiZhuang_Back_y_Window);
}
MenuWindow B_LiZhuang_Back_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "B_LiZhuang_Back_y",			//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.B_LiZhuang_Back_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -60,									//最小值
	.Prob_MaxData = 60, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_B_LiZhuang_Back_y_Window(void){
	OLED_UI_CreateWindow(&B_LiZhuang_Back_y_Window);
}




/*回家相关*/
MenuWindow R_HuiJia_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "R_HuiJia_x",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.R_HuiJia_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -100,									//最小值
	.Prob_MaxData = 100, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_R_HuiJia_x_Window(void){
	OLED_UI_CreateWindow(&R_HuiJia_x_Window);
}
MenuWindow B_HuiJia_x_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "B_HuiJia_x",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.B_HuiJia_x,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -100,									//最小值
	.Prob_MaxData = 100, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_B_HuiJia_x_Window(void){
	OLED_UI_CreateWindow(&B_HuiJia_x_Window);
}
MenuWindow C_HuiJia_y_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "C_HuiJia_y",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间
	.Prob_Data_Int_32 = &STORAGE_Data.C_HuiJia_y,				//显示的变量地址
	.Prob_DataStep = 1,								//步长
	.Prob_MinData = -60,									//最小值
	.Prob_MaxData = 60, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,
};
void Show_C_HuiJia_y_Window(void){
	OLED_UI_CreateWindow(&C_HuiJia_y_Window);
}




















/*上面为示例，下面依照示例创建自己的菜单项变量*/
//主菜单的菜单项

/* ==========================================================================================
   ★★ 以下为本工程原有的 17 个参数窗口 + Show_* 函数（PID Param / Other Param 两个子菜单页用）★★
   来自 2026.10.2 版本工程的 oled_ui_menudata.c，和队友新加的 R_ChuFa_x 等窗口共用同一个 oled_ui 库；
   对应的菜单项数组/菜单页在本文件后半部分（PIDParamMenuItems / PIDParamMenuPage / OtherParamMenuPage）。
   ========================================================================================== */
MenuWindow line_kp_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "line_kp",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.line_kp,				//显示的变量地址
	.Prob_DataStep = 0.05,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示line_kp窗口
 */
void Show_line_kp_Window(void){
	OLED_UI_CreateWindow(&line_kp_Window);
}

MenuWindow line_ki_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "line_ki",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.line_ki,				//显示的变量地址
	.Prob_DataStep = 0.05,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示line_ki窗口
 */
void Show_line_ki_Window(void){
	OLED_UI_CreateWindow(&line_ki_Window);
}

MenuWindow line_kd_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "line_kd",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.line_kd,				//显示的变量地址
	.Prob_DataStep = 0.01,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示line_kd窗口
 */
void Show_line_kd_Window(void){
	OLED_UI_CreateWindow(&line_kd_Window);
}

MenuWindow angle_kp_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "angle_kp",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.angle_kp,				//显示的变量地址
	.Prob_DataStep = 0.05,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示angle_kp窗口
 */
void Show_angle_kp_Window(void){
	OLED_UI_CreateWindow(&angle_kp_Window);
}

MenuWindow angle_ki_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "angle_ki",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.angle_ki,				//显示的变量地址
	.Prob_DataStep = 0.05,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示angle_ki窗口
 */
void Show_angle_ki_Window(void){
	OLED_UI_CreateWindow(&angle_ki_Window);
}

MenuWindow angle_kd_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "angle_kd",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.angle_kd,				//显示的变量地址
	.Prob_DataStep = 0.01,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示angle_kd窗口
 */
void Show_angle_kd_Window(void){
	OLED_UI_CreateWindow(&angle_kd_Window);
}

MenuWindow speed_left_kp_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "speed_left_kp",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.speed_left_kp,				//显示的变量地址
	.Prob_DataStep = 0.05,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示speed_left_kp窗口
 */
void Show_speed_left_kp_Window(void){
	OLED_UI_CreateWindow(&speed_left_kp_Window);
}

MenuWindow speed_left_ki_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "speed_left_ki",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.speed_left_ki,				//显示的变量地址
	.Prob_DataStep = 0.01,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示speed_left_ki窗口
 */
void Show_speed_left_ki_Window(void){
	OLED_UI_CreateWindow(&speed_left_ki_Window);
}

MenuWindow speed_left_kd_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "speed_left_kd",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.speed_left_kd,				//显示的变量地址
	.Prob_DataStep = 0.05,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示speed_left_kd窗口
 */
void Show_speed_left_kd_Window(void){
	OLED_UI_CreateWindow(&speed_left_kd_Window);
}

MenuWindow speed_right_kp_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "speed_right_kp",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.speed_right_kp,				//显示的变量地址
	.Prob_DataStep = 0.05,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示speed_right_kp窗口
 */
void Show_speed_right_kp_Window(void){
	OLED_UI_CreateWindow(&speed_right_kp_Window);
}

MenuWindow speed_right_ki_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "speed_right_ki",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.speed_right_ki,				//显示的变量地址
	.Prob_DataStep = 0.01,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示speed_right_ki窗口
 */
void Show_speed_right_ki_Window(void){
	OLED_UI_CreateWindow(&speed_right_ki_Window);
}

MenuWindow speed_right_kd_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "speed_right_kd",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.speed_right_kd,				//显示的变量地址
	.Prob_DataStep = 0.05,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示speed_right_kd窗口
 */
void Show_speed_right_kd_Window(void){
	OLED_UI_CreateWindow(&speed_right_kd_Window);
}

MenuWindow angle_offset_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "angle_offset",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Float = &STORAGE_Data.angle_offset,				//显示的变量地址
	.Prob_DataStep = 0.02,								//步长
	.Prob_MinData = -5.0,									//最小值
	.Prob_MaxData = 5.0, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示angle_offset窗口
 */
void Show_angle_offset_Window(void){
	OLED_UI_CreateWindow(&angle_offset_Window);
}

MenuWindow encoder_cnt_odd_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "encoder_cnt_odd",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Int_32 = &STORAGE_Data.encoder_cnt_odd,				//显示的变量地址
	.Prob_DataStep = 5,								//步长
	.Prob_MinData = 1500,									//最小值
	.Prob_MaxData = 4500, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示encoder_cnt_odd窗口
 */
void Show_encoder_cnt_odd_Window(void){
	OLED_UI_CreateWindow(&encoder_cnt_odd_Window);
}

MenuWindow encoder_cnt_even_Window = {
	.General_Width = 80,								//窗口宽度
	.General_Height = 28, 							//窗口高度
	.Text_String = "encoder_cnt_even",					//窗口标题
	.Text_FontSize = OLED_UI_FONT_12,				//字高
	.Text_FontSideDistance = 4,							//字体距离左侧的距离
	.Text_FontTopDistance = 3,							//字体距离顶部的距离
	.General_WindowType = WINDOW_ROUNDRECTANGLE, 	//窗口类型
	.General_ContinueTime = 4.0,						//窗口持续时间

	.Prob_Data_Int_32 = &STORAGE_Data.encoder_cnt_even,				//显示的变量地址
	.Prob_DataStep = 5,								//步长
	.Prob_MinData = 1500,									//最小值
	.Prob_MaxData = 4500, 								//最大值
	.Prob_BottomDistance = 3,							//底部间距
	.Prob_LineHeight = 8,								//进度条高度
	.Prob_SideDistance = 4,	
};
/**
 * @brief 创建显示encoder_cnt_even窗口
 */
void Show_encoder_cnt_even_Window(void){
	OLED_UI_CreateWindow(&encoder_cnt_even_Window);
}



MenuItem MainMenuItems[] = {
	{.General_item_text = "Param",.General_callback = NULL,.General_SubMenuPage = &ParamMenuPage,.List_BoolRadioBox = NULL},
	{.General_item_text = "Sensor",.General_callback = NULL,.General_SubMenuPage = NULL,.List_BoolRadioBox = &sensor_data_show},
	{.General_item_text = NULL},/*最后一项的General_item_text置为NULL，表示该项为分割线*/
};



MenuItem ParamMenuItems[] = {
	{.General_item_text = "ChuFa",.General_callback = NULL,.General_SubMenuPage = &Param_ChuFaMenuPage,.List_BoolRadioBox = NULL},
	{.General_item_text = "YuanPan",.General_callback = NULL,.General_SubMenuPage = &Param_YuanPanMenuPage,.List_BoolRadioBox = NULL},
	{.General_item_text = "GetTarget",.General_callback = NULL,.General_SubMenuPage = &Param_GetTargetMenuPage,.List_BoolRadioBox = NULL},
	{.General_item_text = "JieTi",.General_callback = NULL,.General_SubMenuPage = &Param_JieTiMenuPage,.List_BoolRadioBox = NULL},
	{.General_item_text = "LiZhuang",.General_callback = NULL,.General_SubMenuPage = &Param_LiZhuangMenuPage,.List_BoolRadioBox = NULL},
	{.General_item_text = "HuiJia",.General_callback = NULL,.General_SubMenuPage = &Param_HuiJiaMenuPage,.List_BoolRadioBox = NULL},
	{.General_item_text = "exitcancel",.General_callback = NULL,.General_SubMenuPage = NULL,.List_BoolRadioBox = &oled_ui_exit_cancel},
	{.General_item_text = NULL},/*最后一项的General_item_text置为NULL，表示该项为分割线*/

};


MenuItem Param_ChuFaMenuItems[] = {	
	{.General_item_text = "R_ChuFa_x",.General_callback = Show_R_ChuFa_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "R_ChuFa_y",.General_callback = Show_R_ChuFa_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "B_ChuFa_x",.General_callback = Show_B_ChuFa_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "B_ChuFa_y",.General_callback = Show_B_ChuFa_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "exit&save",.General_callback = NULL,.General_SubMenuPage = NULL,.List_BoolRadioBox = &oled_ui_exit_save},
	{.General_item_text = NULL},/*最后一项的General_item_text置为NULL，表示该项为分割线*/
};
MenuItem Param_YuanPanMenuItems[] = {
	{.General_item_text = "R_YuanPan_y",.General_callback = Show_R_YuanPan_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "B_YuanPan_y",.General_callback = Show_B_YuanPan_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "exit&save",.General_callback = NULL,.General_SubMenuPage = NULL,.List_BoolRadioBox = &oled_ui_exit_save},
	{.General_item_text = NULL},/*最后一项的General_item_text置为NULL，表示该项为分割线*/
};
MenuItem Param_GetTargetMenuItems[] = {
	{.General_item_text = "C_GetTarget_y",.General_callback = Show_C_GetTarget_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_GetTarget_x",.General_callback = Show_C_GetTarget_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "exit&save",.General_callback = NULL,.General_SubMenuPage = NULL,.List_BoolRadioBox = &oled_ui_exit_save},
	{.General_item_text = NULL},/*最后一项的General_item_text置为NULL，表示该项为分割线*/
};
MenuItem Param_JieTiMenuItems[] = {
	{.General_item_text = "C_JieTi_Left_x",.General_callback = Show_C_JieTi_Left_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_Jieti_FwdTarget_y",.General_callback = Show_C_Jieti_FwdTarget_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_JitTi_Step_x",.General_callback = Show_C_JitTi_Step_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_Jieti_Cross_x",.General_callback = Show_C_Jieti_Cross_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_JieTi_JiaoZhun_x1",.General_callback = Show_C_JieTi_JiaoZhun_x1_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_JieTi_JiaoZhun_x2",.General_callback = Show_C_JieTi_JiaoZhun_x2_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_JieTi_JiaoZhun_y1",.General_callback = Show_C_JieTi_JiaoZhun_y1_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_JieTi_JiaoZhun_y2",.General_callback = Show_C_JieTi_JiaoZhun_y2_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "exit&save",.General_callback = NULL,.General_SubMenuPage = NULL,.List_BoolRadioBox = &oled_ui_exit_save},
	{.General_item_text = NULL},/*最后一项的General_item_text置为NULL，表示该项为分割线*/
};
MenuItem Param_LiZhuangMenuItems[] = {
	{.General_item_text = "R_LiZhuang_Delay_x",.General_callback = Show_R_LiZhuang_Delay_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "B_LiZhuang_Delay_x",.General_callback = Show_B_LiZhuang_Delay_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_LiZhuang_Target_y",.General_callback = Show_C_LiZhuang_Target_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_LiZhuang_Fwd_y",.General_callback = Show_C_LiZhuang_Fwd_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "R_LiZhuang_Back_y",.General_callback = Show_R_LiZhuang_Back_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "B_LiZhuang_Back_y",.General_callback = Show_B_LiZhuang_Back_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "exit&save",.General_callback = NULL,.General_SubMenuPage = NULL,.List_BoolRadioBox = &oled_ui_exit_save},
	{.General_item_text = NULL},/*最后一项的General_item_text置为NULL，表示该项为分割线*/
};
MenuItem Param_HuiJiaMenuItems[] = {
	{.General_item_text = "R_HuiJia_x",.General_callback = Show_R_HuiJia_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "B_HuiJia_x",.General_callback = Show_B_HuiJia_x_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "C_HuiJia_y",.General_callback = Show_C_HuiJia_y_Window,.General_SubMenuPage = NULL,.List_BoolRadioBox = NULL},
	{.General_item_text = "exit&save",.General_callback = NULL,.General_SubMenuPage = NULL,.List_BoolRadioBox = &oled_ui_exit_save},
};


/*上面为示例，下面依照示例创建自己的菜单页变量*/

MenuPage MainMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = NULL,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = MainMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};


MenuPage ParamMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = &MainMenuPage,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = ParamMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};

MenuPage DebugMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = &MainMenuPage,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = DebugMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};

MenuPage SensorMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = &MainMenuPage,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = SensorMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};




MenuPage Param_ChuFaMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = &ParamMenuPage,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = Param_ChuFaMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};

MenuPage Param_YuanPanMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = &ParamMenuPage,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = Param_YuanPanMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};

MenuPage Param_GetTargetMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = &ParamMenuPage,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = Param_GetTargetMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};
MenuPage Param_JieTiMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = &ParamMenuPage,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = Param_JieTiMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};
MenuPage Param_LiZhuangMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = &ParamMenuPage,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = Param_LiZhuangMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};
MenuPage Param_HuiJiaMenuPage = {
	//通用属性，必填
	.General_MenuType = MENU_TYPE_LIST,  		 //菜单类型为列表类型
	.General_CursorStyle = REVERSE_ROUNDRECTANGLE,	 //光标类型为圆角矩形
	.General_FontSize = OLED_UI_FONT_12,			//字高
	.General_ParentMenuPage = &ParamMenuPage,		 //父菜单
	.General_LineSpace = 6,						//行间距 单位：像素
	.General_MoveStyle = UNLINEAR,				//移动方式为非线性曲线动画
	.General_MovingSpeed = SPEED,					//动画移动速度(此值根据实际效果调整)
	.General_ShowAuxiliaryFunction = NULL,		 //显示辅助函数
	.General_MenuItems = Param_HuiJiaMenuItems,		 //菜单项内容数组

	//特殊属性，根据.General_MenuType的类型选择
	.List_MenuArea = {0, 0, 128, 64},			 //列表显示区域
	.List_IfDrawFrame = true,					 //是否显示边框
	.List_IfDrawLinePerfix = true,				 //是否显示行前缀
	.List_StartPointX = 4,                        //列表起始点X坐标
	.List_StartPointY = 2,                        //列表起始点Y坐标
};