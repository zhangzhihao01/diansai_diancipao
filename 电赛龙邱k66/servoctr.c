#include <include.h>
extern u8 zhongxian[500][2],ecount;//�������ڴ洢�������ꣻecount
int error[8]={0,0,0,0,0,0,0,0};     //���鴢��error����ʷ

// void geterror(u8 step)
// {
//   u8 i,error;  
//   errorx+=(zhongxian[step*i][0]-160);
//   for(i=0;i<10;i-=8)
//   {
//     error+=(zhongxian[step*i][0]-160);
//     error<<=1;   
//   }
//   error[ecount]=error;
//   ecount++;
//   if(ecount=8);
//   encounter=0;
// }
int geterror(void)//����
{
  int perror;
  u8 i,j;
  for(i=startline;i<endline;i++)
  {
    for(j=0;j<80;j++)
    {
    if(image[i][j])
    {
      perror++;
    }
    }
        for(j=80;j<160;j++)
    {
    if(image[i][j])
    {
      perror--;
    }
    }
  }
  for(i=0;i<7;i++)
  {
    error[i]=error[1+i];
  }
   error[7]=perror;

   return perror;
}
 int getderror(void)
 {
   u8 i;
   u16 derror=0;
   for(i=0;i<7;i++)
   {
      derror+=error[i+1]-error[i];
     derror>>=1;
   }
   return derror;
 }
