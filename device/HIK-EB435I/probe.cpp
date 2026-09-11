#include <cstdio>
#include <vector>
#include "Mv3dRgbdApi.h"
static bool okay = true;
static bool check(const char* op, MV3D_RGBD_STATUS s) { printf("%s: 0x%08x\n", op, (unsigned)s); fflush(stdout); okay = okay && s==0; return s==0; }
int main(int argc, char** argv){
 MV3D_RGBD_VERSION_INFO v{};check("version",MV3D_RGBD_GetSDKVersion(&v));printf("SDK %d.%d.%d\n",v.nMajor,v.nMinor,v.nRevision);
 if(!check("initialize",MV3D_RGBD_Initialize()))return 1;
 unsigned n=0;check("count",MV3D_RGBD_GetDeviceNumber(DeviceType_USB,&n));printf("devices=%u\n",n);
 if(!n){MV3D_RGBD_Release();return 2;}
 std::vector<MV3D_RGBD_DEVICE_INFO> devices(n);check("list",MV3D_RGBD_GetDeviceList(DeviceType_USB,devices.data(),n,&n));
 for(auto& d:devices)printf("serial=%s model=%s\n",d.chSerialNumber,d.chModelName);
 void* h=nullptr;
 if(!check("open",MV3D_RGBD_OpenDeviceBySerialNumber(&h,argc > 1 ? argv[1] : "00DA8573794"))){MV3D_RGBD_Release();return 3;}
 for(const char* key:{"CameraWorkingMode","ImageMode"}){
  MV3D_RGBD_PARAM p{}; if(check(key,MV3D_RGBD_GetParam(h,key,&p))) {printf("type=%d current=%u supported:",p.enParamType,p.ParamInfo.stEnumParam.nCurValue);for(unsigned i=0;i<p.ParamInfo.stEnumParam.nSupportedNum;i++)printf(" %u",p.ParamInfo.stEnumParam.nSupportValue[i]);puts("");}
 }
 if(argc > 2) {
  MV3D_RGBD_PARAM p{}; p.enParamType=ParamType_Enum;
  p.ParamInfo.stEnumParam.nCurValue=2;check("set working=2",MV3D_RGBD_SetParam(h,"CameraWorkingMode",&p));
  p.ParamInfo.stEnumParam.nCurValue=8;check("set image=8",MV3D_RGBD_SetParam(h,"ImageMode",&p));
  if(check("start",MV3D_RGBD_Start(h))) {
   for(int i=0;i<5;i++){MV3D_RGBD_FRAME_DATA f{};if(check("fetch",MV3D_RGBD_FetchFrame(h,&f,1000)))for(unsigned j=0;j<f.nImageCount;j++)printf("frame=%u type=%llx %ux%u bytes=%u\n",f.stImageData[j].nFrameNum,(long long)f.stImageData[j].enImageType,f.stImageData[j].nWidth,f.stImageData[j].nHeight,f.stImageData[j].nDataLen);}
   check("stop",MV3D_RGBD_Stop(h));
  }
 }
 check("close",MV3D_RGBD_CloseDevice(&h));check("release",MV3D_RGBD_Release());
 return okay ? 0 : 4;
}
