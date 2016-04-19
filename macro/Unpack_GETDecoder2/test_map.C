void test_map()
{

  TStopwatch timer;
  timer.Start();

  AtTpcMap* fAtMapPtr = new AtTpcMap();
  fAtMapPtr->GenerateATTPC();
  TH2Poly *fPadPlane = fAtMapPtr->GetATTPCPlane();

  Float_t x=0;
  Float_t y=0;
  Float_t z=0;



            for(Int_t i=0;i<100000;i++)
            {

                  x = x + 0.001;
                  y = y + 0.001;
                  z = z + 0.001;
                  Int_t bin=  fPadPlane->Fill(x,y,z);


            }

            fPadPlane->Draw("zcol");

            std::cout << std::endl << std::endl;
            std::cout << "Macro finished succesfully."  << std::endl << std::endl;
            // -----   Finish   -------------------------------------------------------
            timer.Stop();
            Double_t rtime = timer.RealTime();
            Double_t ctime = timer.CpuTime();
            cout << endl << endl;
            cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
            cout << endl;

}
