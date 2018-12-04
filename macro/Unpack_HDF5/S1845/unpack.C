void unpack(){

	TFile* file = new TFile("output_proto.root","READ");

    TTree* tree = (TTree*) file -> Get("cbmsim");
    Int_t nEvents = tree -> GetEntries();
    std::cout<<" Number of events : "<<nEvents<<std::endl;

    TTreeReader Reader1("cbmsim", file);
    TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");

    for(Int_t i=0;i<nEvents;i++){
          

              Reader1.Next();

              std::cout<<i<<"\n";
              ATEvent* event = (ATEvent*) eventArray->At(0);


              if(event!=NULL)
              	{
              		Int_t nHits = event->GetNumHits();
             	 std::cout<<" Number of hits "<<nHits<<"\n";
             	} 
     } 


}