
#include "include.h"

extern float   distance_taget;
extern int     problems;
extern float   speed_v0;  
extern float   shuru_angle_X;
extern float    jianpan_AD_max;                                       //����AD���ֵ
extern float    jianpan_AD_min;                                       //����AD��Сֵ
extern int      k_error;                                               //�������
extern int jiguang_distance; 
uint8_t text1[]={"mode_choice"};
uint8_t text2[]={"1.variate_show(52)"};
uint8_t text3[]={"2.variate_change(53)"};
uint8_t text4[]={"3.course_choice(54)"};
uint8_t text20[]={"variate_show"};
uint8_t text21[]={"1.distance"};
uint8_t text22[]={"2.angle"};
uint8_t text23[]={"3.tfmini"};
uint8_t text24[]={"4.speed"};
uint8_t text30[]={"variate_change"};
uint8_t text31[]={"1.distance(61)"};
uint8_t text32[]={"2.speed(62)"};
uint8_t text33[]={"3.(63)"};
uint8_t text34[]={"4.(64)"};
uint8_t text40[]={"course_choice"};
uint8_t text41[]={"1.mood1(71)"};
uint8_t text42[]={"2.mood2(72)"};
uint8_t text43[]={"3.mood3(73)"};
uint8_t text44[]={"4,mood4(74)"};
uint8_t text311[]={"change_distance:"};
uint8_t text321[]={"change_angle:"};
uint8_t text331[]={"change: max"};
uint8_t text341[]={"change:min"};
uint8_t text411[]={"mood1_ok"};
uint8_t text421[]={"mood2_ok"};
uint8_t text431[]={"mood3_ok"};
uint8_t text441[]={"mood4_ok"};
uint8_t text442[]={"mood5_ok"};
uint8_t text888[]={"change__error:"};
/********************
����1,����ѡ��
********************/
void huamian_1()
{
  TFTSPI_P8X8Str(0,0,text1,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,2,text2,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,4,text3,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,6,text4,u16WHITE,u16BLACK);
}
/********************
����2�������鿴
********************/
void huamian_2()
{
  uint8_t str1[5];
  uint8_t str2[5];
  uint8_t str3[5];
  sprintf(str1 ,"%.2f", distance_taget);
  sprintf(str2 ,"%.2f", shuru_angle_X);
  sprintf(str3 ,"%3d", jiguang_distance);
  TFTSPI_P8X8Str(0,0,text20,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,2,text21,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,4,text22,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,6,text23,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,8,text24,u16WHITE,u16BLACK);
  
  TFTSPI_P8X8Str(12,2,str1,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(12,4,str2,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(12,6,str3,u16WHITE,u16BLACK);

  
}
/********************
����3��ѡ����ı�Ĳ���
********************/
void huamian_3()
{
  TFTSPI_P8X8Str(0,0,text30,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,2,text31,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,4,text32,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,6,text33,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,8,text34,u16WHITE,u16BLACK);
}
/********************
����4��ѡ��ģʽ
********************/
void huamian_4()
{
  TFTSPI_P8X8Str(0,0,text40,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,2,text41,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,4,text42,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,6,text43,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,8,text44,u16WHITE,u16BLACK);
}
/********************
����5���ı����1
********************/
void huamian_5()
{
  uint8_t str[5];
  sprintf(str ,"%.2f", distance_taget);
  
  TFTSPI_P8X8Str(0,0,text311,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,4,str,u16WHITE,u16BLACK);
}
/********************
����6���ı����2
********************/
void huamian_6()
{
  uint8_t str[5];
  sprintf(str ,"%.2f", shuru_angle_X);
  TFTSPI_P8X8Str(0,0,text321,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,4,str,u16WHITE,u16BLACK);
}
/********************
����7���ı����3
********************/
void huamian_7()
{
  uint8_t str[5];
  sprintf(str ,"%.2f", jianpan_AD_max);
  TFTSPI_P8X8Str(0,0,text331,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,4,str,u16WHITE,u16BLACK);
  
}
/********************
����8���ı����4
********************/
void huamian_8()
{
  uint8_t str[5];
  sprintf(str ,"%.2f", jianpan_AD_min);
  TFTSPI_P8X8Str(0,0,text341,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,4,str,u16WHITE,u16BLACK);
}
/********************
����9��ģʽ1
********************/
void huamian_9()
{
  TFTSPI_P8X8Str(0,4,text411,u16WHITE,u16BLACK);
}
/********************
����10��ģʽ2
********************/
void huamian_10()
{
  TFTSPI_P8X8Str(0,4,text421,u16WHITE,u16BLACK);
}
/********************
����11��ģʽ3
********************/
void huamian_11()
{
  TFTSPI_P8X8Str(0,4,text431,u16WHITE,u16BLACK);
}
/********************
����12��ģʽ4
********************/
void huamian_12()
{
  TFTSPI_P8X8Str(0,4,text441,u16WHITE,u16BLACK);
}
/********************
����13��ģʽ5
********************/
void huamian_13()
{
  TFTSPI_P8X8Str(0,4,text442,u16WHITE,u16BLACK);
}
/********************
����14���ı����5
********************/
void huamian_14()
{
  uint8_t str[5];
  sprintf(str ,"%d", k_error);
  TFTSPI_P8X8Str(0,0,text888,u16WHITE,u16BLACK);
  TFTSPI_P8X8Str(0,4,str,u16WHITE,u16BLACK);
}

/******************
��ʾ����
*******************/
void TFT_tuxiangxianshi()
{
  
      static int  i[4]={1,1,1,1};
      static int  j[4]={1,1,1,1};
      static int  k[5]={1,1,1,1,1};
      static int  m[2]={1,1};

       bomakaiguan_function();
       if(problems==10)                             //��ʾ����1
       {
         i[0]=1;
         i[2]=1;
         i[3]=1;
         m[0]=1;
         while(i[1])
         {
           TFTSPI_CLS(u16BLACK);
           i[1]=0;
         }
         huamian_1();
       }
       else if(problems==20)                        //��ʾ����2
       {
         i[1]=1;
         i[0]=1;
         i[3]=1;
         while(i[2])
         {
           TFTSPI_CLS(u16BLACK);
           i[2]=0;
         }
         huamian_2();
       }
       else if(problems==30)                        //��ʾ����3
       {
         i[1]=1;
         i[2]=1;
         i[0]=1;
         while(i[3])
         {
           TFTSPI_CLS(u16BLACK);
           i[3]=0;
         }
         huamian_3();
       }
       else if(problems==40)                       //��ʾ����4
       {
         i[1]=1;
         i[2]=1;
         i[3]=1;
         while(i[0])
         {
           TFTSPI_CLS(u16BLACK);
           i[0]=0;
         }
         huamian_4();
       }
       else if(problems==31||problems==311)                       //��ʾ����5
       {
         j[0]=1;
         j[2]=1;
         j[3]=1;
         while(j[1])
         {
           TFTSPI_CLS(u16BLACK);
           j[1]=0;
         }
         huamian_5();
       }
       else if(problems==32||problems==321)                       //��ʾ����6
       {
         j[1]=1;
         j[0]=1;
         j[3]=1;
         while(j[2])
         {
           TFTSPI_CLS(u16BLACK);
           j[2]=0;
         }
         huamian_6();
       }
       else if(problems==33)                       //��ʾ����7
       {
         j[1]=1;
         j[2]=1;
         j[0]=1;
         while(j[3])
         {
           TFTSPI_CLS(u16BLACK);
           j[3]=0;
         }
         huamian_7();
       }
       else if(problems==34)                       //��ʾ����8
       {
         j[1]=1;
         j[2]=1;
         j[3]=1;
         while(j[0])
         {
           TFTSPI_CLS(u16BLACK);
           j[0]=0;
         }
         huamian_8();
       }
       else if(problems==41)                       //��ʾ����9
       {
         k[0]=1;
         k[2]=1;
         k[3]=1;
         k[4]=1;
         while(k[1])
         {
           TFTSPI_CLS(u16BLACK);
           k[1]=0;
         }
         huamian_9();
       }
        else if(problems==42)                       //��ʾ����10
       {
         k[1]=1;
         k[0]=1;
         k[3]=1;
         k[4]=1;
         while(k[2])
         {
           TFTSPI_CLS(u16BLACK);
           k[2]=0;
         }
         huamian_10();
       }
        else if(problems==43)                       //��ʾ����11
       {
         k[1]=1;
         k[2]=1;
         k[0]=1;
         k[4]=1;
         while(k[3])
         {
           TFTSPI_CLS(u16BLACK);
           k[3]=0;
         }
         huamian_11();
       }
        else if(problems==44)                       //��ʾ����12
       {
         k[1]=1;
         k[2]=1;
         k[3]=1;
         k[4]=1;
         while(k[0])
         {
           TFTSPI_CLS(u16BLACK);
           k[0]=0;
         }
         huamian_12();
       }
        else if(problems==45)                       //��ʾ����13
       {
         k[1]=1;
         k[2]=1;
         k[3]=1;
         k[0]=1;
         while(k[4])
         {
           TFTSPI_CLS(u16BLACK);
           k[4]=0;
         }
         huamian_13();
       }
       else if(problems==331||problems==341)       //��ʾ����14
       {
         while(m[0])
         {
           TFTSPI_CLS(u16BLACK);
           m[0]=0;
         }
         huamian_14();
       }
         
       
}