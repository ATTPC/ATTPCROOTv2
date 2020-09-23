{
   TFile f("CUT1BLA.root");
   TIter next(f.GetListOfKeys());
   TKey *key;
   //cout<<key->GetName()<<endl;
   while ((key=(TKey*)next())) {
      
      cout<<key->GetName()<<endl;
   }
	
}

