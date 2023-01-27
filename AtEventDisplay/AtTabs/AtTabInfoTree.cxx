#include "AtTabInfoTree.h"

#include <TDirectory.h> // for TDirectory

#include <stdexcept> // for invalid_argumet

AtTabInfoTree::AtTabInfoTree(TString tree, TString fileName, DataHandling::AtTreeEntry &entryNumber)
   : AtTabInfoBase(), fReader(tree, TFile::Open(fileName)), fEntryNumber(entryNumber)
{
   // Take ownership of the opened TFile since we are respsonible for deleting the file if opened using
   // TFile::Open() and fReader has to be directly constructed.
   fFile = std::unique_ptr<TFile>(dynamic_cast<TFile *>(fReader.GetTree()->GetDirectory()));

   if (fFile == nullptr)
      throw std::invalid_argument("Could not open " + fileName);
   if (fReader.IsZombie())
      throw std::invalid_argument("Could not find " + tree + " in " + fileName);

   fEntryNumber.Attach(this); // Done at end so we won't leave a dangling referece in the subject
}
AtTabInfoTree::AtTabInfoTree(TTree *tree, DataHandling::AtTreeEntry &entryNumber)
   : AtTabInfoBase(), fReader(tree), fEntryNumber(entryNumber)
{
   if (fReader.IsZombie())
      throw std::invalid_argument("Could not create TTreeReader! Is the passed tree null?");

   fEntryNumber.Attach(this); // Done at end so we won't leave a dangling referece in the subject
}
void AtTabInfoTree::Update(DataHandling::AtSubject *changedSubject)
{
   if (changedSubject == &fEntryNumber)
      fReader.SetEntry(fEntryNumber.Get());
}
