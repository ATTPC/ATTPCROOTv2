#include "S800TSettings.hh"

S800TSettings::S800TSettings(){

  this->fName="TheSettings";
  this->fTitle="S800T's Settings";
  
}

S800TSettings::~S800TSettings(){

}



void S800TSettings::AddCorrectionSettings(string Name,double slope,double inter,double toff){
  TheTimingOffsets[Name] = toff;
  TheEnergySlopes[Name]=slope;
  TheEnergyIntercepts[Name]=inter;

}

void S800TSettings::AddMapSettings(string Name,int GlobalID,string RefName, int refGlobalID){
  GlobalID2FullName[GlobalID] = Name;
  GlobalID2RefGlobalID[GlobalID] = refGlobalID;
  GlobalID2RefName[GlobalID] = RefName;
  
  Name2GlobalID[Name]=GlobalID;
}

void S800TSettings::AddFilterSettings(string Name,int FL,int FG,int d,int w,bool flag){

  TheFLs[Name]=FL;
  TheFGs[Name]=FG;
  Theds[Name]=d;
  Thews[Name]=w;
  TheDontTraceAnalyzeFlags[Name]=flag;
}


void S800TSettings::PrintAll(){

  for (auto & ii : TheTimingOffsets){
    cout<<endl;
    PrintChannelCorrections(ii.first);
    PrintChannelMapInfo(Name2GlobalID[ii.first]);
    PrintFilterInfo(ii.first);
  }

}

void S800TSettings::PrintChannelCorrections(string Name){
  printf("Channel Name %6s has slope %10.4lf has intercept %10.4lf and timming offset %10.4lf\n",Name.c_str(),
	 TheEnergySlopes[Name],TheEnergyIntercepts[Name],TheTimingOffsets[Name]);

}

void S800TSettings::PrintChannelMapInfo(int GlobalID){
  printf("Channel Name %6s has DDAS ID %4d it's reference channel is %6s whith DDAS ID %4d\n",GlobalID2FullName[GlobalID].c_str(),
	 GlobalID,GlobalID2RefName[GlobalID].c_str(),GlobalID2RefGlobalID[GlobalID]);


}
void S800TSettings::PrintFilterInfo(string Name){
  printf("Channel Name %6s has FL:%5d  FG:%5d d:%5d w:%5d NoTraceAnalysis:%5d \n",Name.c_str(),
	 TheFLs[Name],TheFGs[Name],Theds[Name],Thews[Name],TheDontTraceAnalyzeFlags[Name]);

}

void S800TSettings::PrintChannelMapInfo(string Name){
  PrintChannelMapInfo(Name2GlobalID[Name]);
}



void S800TSettings::SetBarIds(map<string,int> v){
  BarIds=v;
  BuildReverseMap();
}

void S800TSettings::BuildReverseMap(){
  for (auto & ii : BarIds){
 
    BarId2Name[ii.second]=ii.first;
  }
}
