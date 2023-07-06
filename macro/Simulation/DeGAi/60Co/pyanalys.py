import matplotlib.pyplot as plt
import numpy as np
import ROOT as r

def gausSim_analysis(num_ev=100000):
    mcFileNameHead = "./DeGAi"
    mcFileNameTail = ".root"
    mcFileName = mcFileNameHead + mcFileNameTail
    outFileNameHead = "./DeGAiana"
    outFileNameTail = ".root"
    outFileName = outFileNameHead + outFileNameTail

    point = r.AtMCPoint()
    pointArray = r.TClonesArray()

    file = r.TFile(mcFileName, "READ")
    tree = file.Get("cbmsim")

    tree.SetBranchAddress("AtMCPoint", r.AddressOf(pointArray))
    nEvents = tree.GetEntriesFast()

    if nEvents > num_ev:
        nEvents = num_ev

    # Histograms
    Energy_loss = r.TH1D("Energy_loss", "Energy_loss", 10000, 0, 10)

    c1 = r.TCanvas()
    c1.Draw()
    Count = 0.0

    for iEvent in range(nEvents):
        tree.GetEvent(iEvent)
        n = pointArray.GetEntries()

        energyLoss = 0.0

        for i in range(n):
            point = r.AtMCPoint(pointArray.At(i))
            VolName = point.GetVolName()
            trackID = point.GetTrackID()

            if VolName.Contains("Crystal_"):
                print("Volume Name:", VolName)

                fResolutionGe = 0.30
                inputEnergy = point.GetEnergyLoss()
                randomIs = r.gRandom.Gaus(
                    0, inputEnergy * fResolutionGe * 1000 / (235 * np.sqrt(inputEnergy * 1000))
                )
                energyLoss += (inputEnergy + randomIs / 1000) * 1000  # MeV
                Count += 1

        if energyLoss != 0.0:
            Energy_loss.Fill(energyLoss)

    # Photo Peak efficiency
    momentum = 0.005
    photopeakcount = 0.0
    for i in range(1000):
        cur = Energy_loss.GetBinContent(i)
        if cur > photopeakcount:
            photopeakcount = cur

    eff = (photopeakcount * 100 / Count)
    print("Photo peak efficiency:", eff)

    c1.cd(1)
    r.gStyle.SetOptStat(0)
    Energy_loss.Draw()

gausSim_analysis()

