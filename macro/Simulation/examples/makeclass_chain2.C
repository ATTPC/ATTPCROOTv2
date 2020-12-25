{
TChain chain("cbmsim") ;
chain.Add("data/attpcsim_proto.root");
chain.GetListOfFiles()->Print();
chain.MakeClass("Test_Ana_d2He");
} 
