/*********************************************************************
*   ATTPC Mapping Class	AtTpcMap.cxx			             *
*   Author: Y. Ayyad            				     *
*   Log: 13-02-2015 17:16 JST					     *
*								     *
*********************************************************************/

#include "AtTpcMap.h"

#include <iostream>
#include <cassert>


using std::cout;
using std::endl;

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"


AtTpcMap::AtTpcMap():AtPadCoord(boost::extents[10240][3][2])
{
 Initialize();
}

AtTpcMap::~AtTpcMap()
{


  //delete cATTPCPlane;
  //delete hPlane;


}

void AtTpcMap::Initialize()
{

  kIsParsed=0;
  kGUIMode=0;
  kDebug=0;
  std::fill( AtPadCoord.data(), AtPadCoord.data()+AtPadCoord.num_elements() , 0);
  std::cout<<" ATTPC Map initialized "<<std::endl;
  std::cout<<" ATTPC Pad Coordinates container initialized "<<std::endl;
  fPadInd = 0;
  PadKey.clear();
  fIniPads.clear();
}

void AtTpcMap::Dump(){

    int values = 0;
  	 for(index i = 0; i != 10240; ++i)
   	   for(index j = 0; j != 3; ++j)
     	     for(index k = 0; k != 2; ++k)
             	std::cout<<" ATTPC Triangular pad coordinates - Pad Index : "<<i<<"   X("<<j<<")  -  Y("<<k<<") :"<<AtPadCoord[i][j][k]<<std::endl;

}

void AtTpcMap::GenerateATTPC(){


    std::cout<<" ATTPC Map : Generating the map geometry of the ATTPC "<<std::endl;
    // Local variables
    Float_t pads_in_half_hex;
    Float_t pads_in_hex;
    Float_t row_length = 0;
    Float_t pads_in_half_row = 0;
    Int_t pads_out_half_hex = 0 ;
    Int_t pads_in_row = 0;
    Int_t ort = 0;
    Float_t pad_x_off = 0;
    Float_t pad_y_off = 0;
    Float_t tmp_pad_x_off = 0;
    Float_t tmp_pad_y_off = 0;

    Float_t small_z_spacing = 2*25.4/1000.;
    Float_t small_tri_side = 184.*25.4/1000.;
    Double_t umega_radius = 10826.772*25.4/1000.;
    Float_t beam_image_radius = 4842.52*25.4/1000.;
    Int_t pad_index = 0;
    Int_t pad_index_aux=0;

    Float_t small_x_spacing = 2. * small_z_spacing / TMath::Sqrt(3.);
    Float_t small_y_spacing = small_x_spacing * TMath::Sqrt(3.);
    Float_t dotted_s_tri_side = 4. * small_x_spacing + small_tri_side;
    Float_t dotted_s_tri_hi = dotted_s_tri_side * TMath::Sqrt(3.)/2.;
    Float_t dotted_l_tri_side = 2. * dotted_s_tri_side;
    Float_t dotted_l_tri_hi = dotted_l_tri_side * TMath::Sqrt(3.)/2.;
    Float_t large_x_spacing = small_x_spacing;
    Float_t large_y_spacing = small_y_spacing;
    Float_t large_tri_side = dotted_l_tri_side - 4.*large_x_spacing;
    Float_t large_tri_hi = dotted_l_tri_side * TMath::Sqrt(3.)/2.;
    //Float_t row_len_s = 2**TMath::Ceil(TMath::Log(beam_image_radius/dotted_s_tri_side)/TMath::Log(2.0));
    Float_t row_len_s = pow(2,TMath::Ceil(TMath::Log(beam_image_radius/dotted_s_tri_side)/TMath::Log(2.0)));
    Float_t row_len_l = TMath::Floor(umega_radius/dotted_l_tri_hi);

    Float_t xoff = 0.;
    Float_t yoff = 0.;



    for(Int_t j =0;j<row_len_l;j++){

        pads_in_half_hex = 0;
        pads_in_hex = 0;
        //row_length = TMath::Abs(sqrt(umega_radius**2 - (j*dotted_l_tri_hi + dotted_l_tri_hi/2.)**2));
        row_length = TMath::Abs(sqrt(pow(umega_radius,2) - pow((j*dotted_l_tri_hi + dotted_l_tri_hi/2.),2)));

        if(j<row_len_s/2.){

            pads_in_half_hex = (2*row_len_s - 2*j)/4.;
            pads_in_hex = 2*row_len_s - 1. - 2.*j;


        }//if row_len_s

        pads_in_half_row = row_length /dotted_l_tri_side;
        pads_out_half_hex = static_cast<Int_t> (TMath::Nint(2*(pads_in_half_row-pads_in_half_hex)));
        pads_in_row = 2 * pads_out_half_hex + 4 * pads_in_half_hex - 1;

	ort = 1;



            for(Int_t i =0;i<pads_in_row;i++){

                if(i==0){

                    if(j%2==0) ort = -1;
                    if(( (pads_in_row-1)/2)%2 == 1) ort = -ort;

                }//i==0
                else ort = -ort;



                pad_x_off = -(pads_in_half_hex + pads_out_half_hex/2.)*dotted_l_tri_side + i*dotted_l_tri_side/2. + 2.*large_x_spacing + xoff;

                if(i<pads_out_half_hex || i> (pads_in_hex + pads_out_half_hex - 1) || j> (row_len_s/2. -1)){

                    pad_y_off = j*dotted_l_tri_hi + large_y_spacing + yoff;
                    if(ort==-1) pad_y_off+=large_tri_hi;
                    fill_coord(pad_index,pad_x_off,pad_y_off,large_tri_side,ort);
                    pad_index+=1;


                }//if
                else{

                    pad_y_off = j*dotted_l_tri_hi + large_y_spacing + yoff;
                    if(ort==-1) pad_y_off = j*dotted_l_tri_hi + 2*dotted_s_tri_hi - small_y_spacing+yoff;
                    fill_coord(pad_index,pad_x_off, pad_y_off, small_tri_side, ort);
                    pad_index += 1;
                    tmp_pad_x_off = pad_x_off + dotted_s_tri_side/2.;
                    tmp_pad_y_off = pad_y_off + ort*dotted_s_tri_hi - 2*ort*small_y_spacing;
                    fill_coord(pad_index,tmp_pad_x_off, tmp_pad_y_off, small_tri_side, -ort);
                    pad_index += 1;
                    tmp_pad_y_off = pad_y_off+ort*dotted_s_tri_hi;
                    fill_coord(pad_index,tmp_pad_x_off, tmp_pad_y_off, small_tri_side, ort);
                    pad_index += 1;
                    tmp_pad_x_off = pad_x_off + dotted_s_tri_side;
                    fill_coord(pad_index,tmp_pad_x_off, pad_y_off, small_tri_side, ort);
                    pad_index += 1;


                }


        }//pads_in_row loop


    }//row_len_l

	   for(Int_t i =0;i<pad_index;i++){
        	for(Int_t j=0;j<3;j++){
           		 AtPadCoord[i+pad_index][j][0]=AtPadCoord[i][j][0];
           		 AtPadCoord[i+pad_index][j][1]=-AtPadCoord[i][j][1];


       		 }
      		  pad_index_aux++;
   	 }



       fPadInd = pad_index + pad_index_aux;

}


Int_t AtTpcMap::fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort){


    AtPadCoord[pindex][0][0] = padxoff;
    AtPadCoord[pindex][0][1] = padyoff;
    AtPadCoord[pindex][1][0] = padxoff + triside/2.;
    AtPadCoord[pindex][1][1] = padyoff + fort*triside*TMath::Sqrt(3.)/2.;
    AtPadCoord[pindex][2][0] = padxoff + triside;
    AtPadCoord[pindex][2][1] = padyoff;


}

TH2Poly* AtTpcMap::GetATTPCPlane(){

      if(fPadInd == 0){

	std::cout<<" AtTpcMap::GetATTPCPlane Error : Pad plane has not been generated - Exiting... "<<std::endl;

	return 0;

      }



        hPlane->SetName("ATTPC_Plane");
    	  hPlane->SetTitle("ATTPC_Plane");

		 for(Int_t i=0;i<fPadInd;i++){

      			  Double_t px[]={AtPadCoord[i][0][0],AtPadCoord[i][1][0],AtPadCoord[i][2][0]};
      			  Double_t py[]={AtPadCoord[i][0][1],AtPadCoord[i][1][1],AtPadCoord[i][2][1]};
       			  hPlane->AddBin(3,px,py);

       		 }

       if(kGUIMode){
        cATTPCPlane = new TCanvas("cATTPCPlane","cATTPCPlane",1000,1000);
        gStyle->SetPalette(1);
    	  hPlane->Draw("col");
       }

       return hPlane;
}

Bool_t AtTpcMap::ParseXMLMap(Char_t const *xmlfile){


	    TDOMParser *domParser = new TDOMParser();
	    domParser->SetValidate(false);
	    Int_t parsecode = domParser->ParseFile(xmlfile);
	    if(parsecode<0){
		std::cerr<< domParser->GetParseCodeMessage(parsecode) <<std::endl;
                return false;
	    }
	    TXMLNode * node = domParser->GetXMLDocument()->GetRootNode();
	    ParseMapList(node->GetChildren());
	    //itrEnd = pmap.end();
	    delete domParser;

	   return true;

}

void AtTpcMap::ParseMapList(TXMLNode *node){

		/*std::vector<int> test;
                test.resize(4);
                test[0] = 0;
                test[1] = 0;
                test[2] = 0;
                test[3] = 0;*/


	for(; node;node=node->GetNextNode()){
	 if(node->GetNodeType()==TXMLNode::kXMLElementNode){ //Element node
	    if(strcmp(node->GetNodeName(),"Lookup20150611") == 0 || strcmp(node->GetNodeName(),"LookupProto20150331") == 0 || strcmp(node->GetNodeName(),"LookupProto10Be") == 0){ //TODO Implement this as function parameter ( I Know this is very dirty
				//cout<<node->GetNodeName()<<endl;
                                //if(strcmp(node->GetNodeName(),"Lookup20141208") == 0){
				ParseATTPCMap(node->GetChildren());
			   }
             else std::cout<<" AtTpcMap::ParseMapList - Node not found! Check node name"<<std::endl;
			}

		}

    		kIsParsed=1;


}


void AtTpcMap::ParseATTPCMap(TXMLNode *node){

		Int_t fCoboID=-1000;
    Int_t fAsadID=-1000;
		Int_t fAgetID=-1000;
		Int_t fChannelID=-1000;
    Int_t fPadID=-1000;



		for ( ; node; node = node->GetNextNode()) {
   		 	 if (node->GetNodeType() == TXMLNode::kXMLElementNode) { // Element Node
				if (strcmp(node->GetNodeName(), "CoboID") == 0)
				fCoboID = atoi(node->GetText());
				if (strcmp(node->GetNodeName(), "AsadID") == 0)
				fAsadID = atoi(node->GetText());
				if (strcmp(node->GetNodeName(), "AgetID") == 0)
				fAgetID = atoi(node->GetText());
       	if (strcmp(node->GetNodeName(), "ChannelID") == 0)
				fChannelID = atoi(node->GetText());
				if (strcmp(node->GetNodeName(), "PadID") == 0)
				fPadID = atoi(node->GetText());


			}



		}

		PadKey.push_back(fCoboID);
    PadKey.push_back(fAsadID);
    PadKey.push_back(fAgetID);
		PadKey.push_back(fChannelID);

		ATTPCPadMap.insert(std::pair<std::vector<int>,int>(PadKey,fPadID));
                //ATTPCPadMap.insert(std::pair<int,std::vector<int>>(fPadID,PadKey));
		PadKey.clear();
		if(kDebug) cout<<"PadID : "<<fPadID<<" - CoboID : "<<fCoboID<<"  - AsadID : "<<fAsadID<<"  - AgetID : "<<fAgetID<<"  - ChannelID : "<<fChannelID<<endl;


}



Bool_t AtTpcMap::DumpATTPCMap(){

			//Option 1: Int key - vector<int> value
		/*	std::map<int,std::vector<int>>::iterator it;
               		std::ostream_iterator<int> ii (std::cout,", ");

				for(it=this->ATTPCPadMap.begin(); it!=this->ATTPCPadMap.end(); ++it){
					std::cout<<" [ "<<(*it).first<<", ";
					std::copy ((*it).second.begin(), (*it).second.end(), ii );
					std::cout<<"]"<<std::endl;;

				}

			std::map<int, std::vector<int>>::const_iterator ite = ATTPCPadMap.find(1);
                        std::string value = it->second;
			*/

		 //Option 2: vector<int> key - int value


		  if(!fPadInd || !kIsParsed){

			std::cout<<" AtTpcMap::DumpATTPCMap Error : Pad plane has not been generated or parsed - Exiting... "<<std::endl;

			return false;

      		  }

			std::map<std::vector<int>,int>::iterator it;
      std::ostream_iterator<int> ii (std::cout,", ");

				for(it=this->ATTPCPadMap.begin(); it!=this->ATTPCPadMap.end(); ++it){
					std::cout<<" [ "<<(*it).second<<", ";
					std::copy ((*it).first.begin(), (*it).first.end(), ii );
					std::cout<<"]"<<std::endl;;

				}

			return true;


}

Int_t AtTpcMap::GetPadNum(std::vector<int> PadRef){ //TODO Better to pass a pointer!


                PadRef.resize(4);
               /* PadRef[0] = 0;
                PadRef[1] = 0;
                PadRef[2] = 0;
                PadRef[3] = 0;*/

		//for(int i=0;i<4;i++) std::cout<<PadRef[i]<<endl;

			//Option 1: Int key - vector<int> value
			//std::map<int, std::vector<int>>::const_iterator ite = ATTPCPadMap.find(1);
                        //std::string value = it->second;
			 //Option 2: vector<int> key - int value
			 std::map<std::vector<int>,int>::const_iterator its =ATTPCPadMap.find(PadRef);
			 int value = (*its).second;
			 //std::cout<<int(ATTPCPadMap.find(test) == ATTPCPadMap.end())<<endl;
      Int_t kIs = int(ATTPCPadMap.find(PadRef) == ATTPCPadMap.end());
            if(kIs){
                    if(kDebug) std::cerr<<" AtTpcMap::GetPadNum - Pad key not found - CoboID : "<<PadRef[0]<<"  AsadID : "<<PadRef[1]<<"  AgetID : "<<PadRef[2]<<"  ChannelID : "<<PadRef[3]<<endl;
                    return -1;
             }
			 //std::cout<<value<<std::endl;
			 //std::map<std::vector<int>,int>::const_iterator its;
			 //std::cout<<ATTPCPadMap.find(test)->second<<std::endl;
			// auto its = ATTPCPadMap.find(test);
			// std::cout << "x: " << (int)its->second << "\n";


			       else return value;


		/*for (auto& m : ATTPCPadMap){ //C+11 style

			for(auto& kv : m.second){
				std::cout<<m.first<<'\n';
			}
		}*/





}

std::vector<Float_t> AtTpcMap::CalcPadCenter(Int_t PadRef){

             std::vector<Float_t> PadCenter={-9999,-9999};
             PadCenter.reserve(2);


	      if(!fPadInd || !kIsParsed){

		    std::cout<<" AtTpcMap::CalcPadCenter Error : Pad plane has not been generated or parsed "<<std::endl;
		    return PadCenter;

      	}

   	   if(PadRef!=-1){ //Boost multi_array crashes with a negative index


               Float_t x = (AtPadCoord[PadRef][0][0] + AtPadCoord[PadRef][1][0] + AtPadCoord[PadRef][2][0])/3.;
               PadCenter[0]=x;
               Float_t y = (AtPadCoord[PadRef][0][1] + AtPadCoord[PadRef][1][1] + AtPadCoord[PadRef][2][1])/3.;
	             PadCenter[1]=y;
               return PadCenter;

	   }else{

		if(kDebug) std::cout<<" AtTpcMap::CalcPadCenter Error : Pad not found"<<std::endl;
		return PadCenter;

	   }

}

Bool_t AtTpcMap::ParseInhibitMap(TString inimap, TString lowgmap, TString xtalkmap)
{

     std::ifstream *fIni = new std::ifstream(inimap.Data());
     std::ifstream *fLowg = new std::ifstream(lowgmap.Data());
     std::ifstream *fXtalk =new std::ifstream(xtalkmap.Data());

     Int_t pad;
     Bool_t IsIniPar = kTRUE;
     Bool_t IsLowgPar = kTRUE;
     Bool_t IsXtalkPar = kTRUE;

      std::cout<< cYELLOW <<__func__ <<" - Parsing map for inhibited pads "<<cNORMAL<<std::endl;

     if(fIni->fail()){
       std::cout<< cRED <<__func__ <<" = Warning : No Inhibit Pad Map found! Please, check the path. Current :"<<inimap.Data()<<cNORMAL<<std::endl;
       IsIniPar = kFALSE;
     }
     if(fLowg->fail()){
       std::cout<< cRED <<__func__ <<" = Warning : No Inhibit Pad Map found! Please, check the path. Current :"<<lowgmap.Data()<<cNORMAL<<std::endl;
       IsLowgPar = kFALSE;
     }
     if(fXtalk->fail()){
       std::cout<< cRED <<__func__ <<" = Warning : No Inhibit Pad Map found! Please, check the path. Current :"<<xtalkmap.Data()<<cNORMAL<<std::endl;
       IsXtalkPar = kFALSE;
     }

            while(!fIni->eof() && IsIniPar){
              *fIni>>pad;
              fIniPads.insert(pad);
            }

            while(!fLowg->eof() && IsLowgPar){
              *fLowg>>pad;
              fIniPads.insert(pad);
            }

            while(!fXtalk->eof() && IsXtalkPar){
              *fXtalk>>pad;
              fIniPads.insert(pad);
            }

         std::cout<< cYELLOW <<__func__ <<"     "<<fIniPads.size()<<" pads added to inhibition list"<<cNORMAL<<std::endl;

     return kTRUE;


}

Bool_t AtTpcMap::GetIsInhibited(Int_t PadNum){
        //std::find(fIniPads.begin(),fIniPads.end(),PadNum);
        const Bool_t kIsInhibit = fIniPads.find(PadNum) != fIniPads.end();
        return kIsInhibit;
}


ClassImp(AtTpcMap)
