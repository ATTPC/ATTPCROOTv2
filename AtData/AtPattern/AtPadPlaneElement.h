#ifndef ATPADPLANELEMENT_H
#define ATPADPLANELEMENT_H

#include <TArc.h>
#include <TObject.h> // for TObject

#include <memory>

/**
 * @brief Base class for elements that can be drawn on the pad plane.
 *
 * These will share the same base elements as normal drawable classes.
 */
class AtPadPlaneElement {
public:
   AtPadPlaneElement() = default;
   virtual ~AtPadPlaneElement() = default;

   virtual void Draw() const = 0;
   virtual void SetLineColor(Color_t color) = 0;
   virtual void SetLineWidth(Width_t width) = 0;
   virtual void SetLineStyle(Style_t style) = 0;
   virtual void SetFillStyle(Style_t style) = 0;
};

class AtPadPlaneCircle : public AtPadPlaneElement {
protected:
   std::unique_ptr<TArc> fCircle;

public:
   AtPadPlaneCircle(float x, float y, float r) : fCircle(std::make_unique<TArc>(x, y, r)) {}
   virtual ~AtPadPlaneCircle() = default;

   virtual void Draw() const override { fCircle->Draw("SAME"); }
   virtual void SetLineColor(Color_t color) override { fCircle->SetLineColor(color); }
   virtual void SetLineWidth(Width_t width) override { fCircle->SetLineWidth(width); }
   virtual void SetLineStyle(Style_t style) override { fCircle->SetLineStyle(style); }
   virtual void SetFillStyle(Style_t style) override { fCircle->SetFillStyle(style); }
};

#endif // #ifndef ATPADPLANELEMENT_H
