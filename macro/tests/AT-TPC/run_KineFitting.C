void run_KineFitting()
{

  //

  // Deuteron
  //Ex1
  // 3D position: TVector3 A 3D physics vector (x,y,z)=(0.154338,-0.475520,39.808893) (rho,theta,phi)=(39.812032,0.719511,-72.018279)
  // 3D momentum: TVector3 A 3D physics vector (x,y,z)=(0.032863,-0.133326,0.136415) (rho,theta,phi)=(0.193558,45.188715,-76.153215)
  //Ex2
  //3D position: TVector3 A 3D physics vector (x,y,z)=(1.383145,-0.035513,37.884856) (rho,theta,phi)=(37.910113,2.091582,-1.470772)
  //3D momentum: TVector3 A 3D physics vector (x,y,z)=(0.114004,-0.010285,0.107284) (rho,theta,phi)=(0.156885,46.855305,-5.155180)


  //10Be
  //Ex 1
  // 3D position: TVector3 A 3D physics vector (x,y,z)=(-0.057949,0.478828,43.106549) (rho,theta,phi)=(43.109247,0.641060,96.900540)
  // 3D momentum: TVector3 A 3D physics vector (x,y,z)=(-0.021864,0.136026,1.214899) (rho,theta,phi)=(1.222686,6.469810,99.131283)
  //Ex 2
  //3D position: TVector3 A 3D physics vector (x,y,z)=(-0.442648,0.047374,40.946464) (rho,theta,phi)=(40.948884,0.622903,173.891280)
 //3D momentum: TVector3 A 3D physics vector (x,y,z)=(-0.116263,0.013200,1.256397) (rho,theta,phi)=(1.261834,5.320676,173.522828)



  Double_t m_p = 1.007825 * 931.49401;
  Double_t m_d = 2.0135532 * 931.49401;
  Double_t m_He3 = 3.016029 * 931.49401;
  Double_t m_Be10 = 10.013533818 * 931.49401;
  Double_t m_Be11 = 11.021657749 * 931.49401;
  Double_t m_Li9 = 9.026790 * 931.49401;
  Double_t m_beam = m_Be10;
  Float_t aMass = 4.00260325415;
  Float_t O16Mass = 15.99491461956;

  Double_t m_a = 4.00260325415 * 931.49401;
  Double_t m_O16 = 15.99491461956 * 931.49401;

  Double_t Ebeam_buff = 99.0; // 90.5;
  Double_t m_b;
  Double_t m_B;

   m_b = m_d;
   m_B = m_Be10;

   //Double_t vertex = 39.808893;//Ex1
   Double_t vertex = 37.88;//Ex2
   Double_t eloss = 0.11269*vertex - 0.05228; //adhoc function to get the energy at the vertex of a 10Be beam at 10A MeV in 600 torr D2. In cm
   Double_t evertex = Ebeam_buff - eloss;
   std::cout<<" Beam energy at vertex "<<evertex<<"\n";

   ROOT::Math::XYZVector beamMom(0,0,TMath::Sqrt(evertex * evertex + 2.0 * evertex * m_B));
   Double_t beamEtot = evertex + m_B;
   std::cout<<" Beam momentum "<<sqrt(beamMom.Mag2())<<"\n";

   //ROOT::Math::XYZVector momSca(0.032863*1000.0,-0.133326*1000.0,0.136415*1000.0);//Ex1
   //Double_t EScaTot = 9.96088 + m_d;//Ekin + mass Ex1
   ROOT::Math::XYZVector momSca(0.114004*1000,-0.010285*1000,0.107284*1000);//Ex2
   Double_t EScaTot = 6.58223 + m_d;//Ekin + mass Ex1
   std::cout<<" Scattered momentum "<<sqrt(momSca.Mag2())<<"\n";
   std::cout<<" Scattered energy (from momentum) "<<sqrt(momSca.Mag2() + m_d*m_d)-m_d<<"\n";

   //ROOT::Math::XYZVector momRec(-0.021864*1000.0,0.136026*1000.0,1.214899*1000.0);//Ex1
   //Double_t ERecTot =  79.7955 + m_Be10;//Ekin + mass Ex1
   ROOT::Math::XYZVector momRec(-0.116263*1000,0.013200*1000,1.256397*1000);//Ex1
   Double_t ERecTot = 85.596 + m_Be10;//Ekin + mass Ex2
   std::cout<<" Recoil momentum "<<sqrt(momRec.Mag2())<<"\n";
   std::cout<<" Recoil energy (from momentum) "<<sqrt(momRec.Mag2() + m_Be10*m_Be10)-m_Be10<<"\n";

   std::cout<<"\n";
   std::cout<<" Momentum in : "<<"\n";
   std::cout<<"  "<<beamMom.X()<<" "<<beamMom.Y()<<" "<<beamMom.Z()<<" "<<beamEtot<<" "<<momSca.X()<<" "<<momSca.Y()<<" "<<momSca.Z()<<" "<<EScaTot<<" "<<momRec.X()<<" "<<momRec.Y()<<" "<<momRec.Z()<<" "<<ERecTot<<"\n";

   std::vector<Double_t> parameters{beamMom.X(),beamMom.Y(),beamMom.Z(),beamEtot,momSca.X(),momSca.Y(),momSca.Z(),EScaTot,momRec.X(),momRec.Y(),momRec.Z(),ERecTot};
   AtTools::AtKinematics kinematics;
   auto fitParameters = kinematics.KinematicalFit(parameters);

   std::cout<<" Momentum out : "<<"\n";
   for(auto fpar : fitParameters)
     std::cout<<"  "<<fpar;

   std::cout<<"\n";
}
