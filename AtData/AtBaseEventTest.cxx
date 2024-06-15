#include "AtBaseEvent.h"

#include <gtest/gtest.h>

TEST(AtBaseEventTest, Clear_Event)
{
   AtBaseEvent event;
   event.Clear(nullptr);
   //  Add assertions to check if the event is cleared properly
   EXPECT_EQ(event.GetEventID(), -1);
   EXPECT_EQ(event.IsGood(), true);
   EXPECT_EQ(event.GetIsExtGate(), false);
   EXPECT_EQ(event.GetEventName(), "");
   EXPECT_EQ(event.GetAuxPads().size(), 0);
}

TEST(AtBaseEventTest, AddAuxPad_InsertUnique)
{
   AtBaseEvent event;
   std::string auxName = "AuxPad1";
   std::pair<AtAuxPad *, bool> result = event.AddAuxPad(auxName);

   EXPECT_TRUE(result.second);
   EXPECT_TRUE(result.first != nullptr);
   EXPECT_EQ(event.GetAuxPads().size(), 1);
}

TEST(AtBaseEventTest, AddAuxPad_InsertDuplicate)
{
   AtBaseEvent event;
   std::string auxName = "AuxPad1";
   event.AddAuxPad(auxName);
   std::pair<AtAuxPad *, bool> result = event.AddAuxPad(auxName);

   EXPECT_FALSE(result.second);
   EXPECT_TRUE(result.first != nullptr);
   EXPECT_EQ(event.GetAuxPads().size(), 1);
}

TEST(AtBaseEventTest, GetAuxPad_Exists)
{
   AtBaseEvent event;
   std::string auxName = "AuxPad1";
   event.AddAuxPad(auxName);
   const AtAuxPad *auxPad = event.GetAuxPad(auxName);

   EXPECT_TRUE(auxPad != nullptr);
}

TEST(AtBaseEventTest, GetAuxPad_DoesNotExist)
{
   AtBaseEvent event;
   const AtAuxPad *auxPad = event.GetAuxPad("");

   EXPECT_TRUE(auxPad == nullptr);
}

TEST(AtBaseEventTest, SetTimestamp_InvalidIndex)
{
   AtBaseEvent event;
   ULong64_t timestamp = 1234567890;
   int index = 1;

   // TODO: Add a class to capture the log messages and make sure the error message is logged
   event.SetTimestamp(timestamp, index);

   EXPECT_EQ(event.GetTimestamp(index), 0);
}

TEST(AtBaseEventTest, SetTimestamp_ValidIndex)
{
   AtBaseEvent event;
   ULong64_t timestamp = 1234567890;
   int index = 1;
   event.SetNumberOfTimestamps(2);
   event.SetTimestamp(timestamp, index);
   // Add assertions to check if the timestamp is set correctly
   EXPECT_EQ(event.GetTimestamp(index), timestamp);
}