#ifndef ATTRACKFINDER_H
#define ATTRACKFINDER_H

#include <vector>
#include "AtTrack.h"

class AtTrackFinder {
public:
    bool MergeTracks(std::vector<AtTrack*>* candToMergePool, std::vector<AtTrack>* mergedTrackPool, bool enableSingleVertexTrack, double clusterRadius, double clusterSize);
    // Other methods and members...
};

#endif // ATTRACKFINDER_H