#include "AtSpaceChargeModel.h"

#include <Math/Point3D.h>
#include <Math/Vector3D.h>

using XYZPoint = ROOT::Math::XYZPoint;

XYZPoint AtSpaceChargeModel::OffsetForBeam(XYZPoint point)
{
   XYZPoint fOffset = fWindow + (fPadPlane - fWindow) / (fPadPlane.Z() - fWindow.Z()) * point.Z();

   return {point.X() - fOffset.X(), point.Y() - fOffset.Y(), point.Z()};
}
XYZPoint AtSpaceChargeModel::UndoOffsetForBeam(XYZPoint point)
{
   XYZPoint fOffset = fWindow + (fPadPlane - fWindow) / (fPadPlane.Z() - fWindow.Z()) * point.Z();

   return {point.X() + fOffset.X(), point.Y() + fOffset.Y(), point.Z()};
}
