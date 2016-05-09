#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <string>

#include "EventLoop.hh"

int target_thread_num = 4;

void SignalHandler(int s);
vector<string> GetInputFiles(const char* inputfile);

Int_t main(int argc, char** argv) {

  // install the interupt handler, to prevent an MPI hayday when
  // ctrl-c killing a local mpiexec job
  struct sigaction InteruptHandler;
  InteruptHandler.sa_handler = SignalHandler;
  sigemptyset(&InteruptHandler.sa_mask);
  InteruptHandler.sa_flags = 0;
  sigaction(SIGINT, &InteruptHandler, NULL);

  int MPIThreadProvision, rank;
  MPI_Init_thread(&argc, &argv,MPI_THREAD_SINGLE,&MPIThreadProvision);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  gSystem->Load("libATTPCReco.so");

  string workdir = getenv("VMCWORKDIR");
  string FileNameHead = "output";
  string FilePath = workdir + "/macro/Unpack_GETDecoder2/";
  string FileNameTail = ".root";
  string FileName     = FilePath + FileNameHead + FileNameTail;

  //EventLoop analyzer("cbmsim",GetInputFiles("./inputfiles.dat"));
  EventLoop analyzer("cbmsim",{FileName});
  if (argc > 1) {   analyzer.SetOutputPath(argv[1]);    }


  TStopwatch timer;
  timer.Start();
  ///////////////
  analyzer.Run();
  ///////////////
  timer.Stop();
  Double_t rtime = timer.RealTime();
  Double_t ctime = timer.CpuTime();

  // record min time accross all ranks (minimum is the most reliable measurement)
  MPI_Allreduce(MPI_IN_PLACE, &rtime, 1, MPI_DOUBLE, MPI_MIN, MPI_COMM_WORLD);
  MPI_Allreduce(MPI_IN_PLACE, &ctime, 1, MPI_DOUBLE, MPI_MIN, MPI_COMM_WORLD);

  if (rank == 0) {
    std::cout << std::endl << std::endl;
    std::cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << std::endl;
    std::cout << std::endl;
  }


  return 0;

}



void SignalHandler(int s) {
  int size; MPI_Comm_size(MPI_COMM_WORLD, &size);
  cout << "Caught signal: "<< s << ". Shutting down.." << endl;
  if (size > 1) {
    MPI_Barrier(MPI_COMM_WORLD);
    MPI_Finalize();
  }
  exit(1);
}

vector<string> GetInputFiles(const char* inputfile) {
  ifstream input(inputfile);
  string fileline;
  vector<string> inputfiles;
  while(input >> fileline) {
    if (fileline.size() > 0)inputfiles.push_back(fileline);
  }
  return inputfiles;
}
