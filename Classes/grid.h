#pragma once
#include "figure.h"

class Grid : public Figure
{
public:
  float size;
  int division;

  Grid(float sizeN = 300.f, int divisionN = 100);

  void Render(int colorLoc) override;

private:
  std::tuple<std::vector<GLfloat>, std::vector<GLuint>> Calculate() const;
  std::tuple<std::vector<GLfloat>, std::vector<GLuint>>
  InitializeAndCalculate(float size, int division);
};