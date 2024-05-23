#ifndef MOVEIT_EXTENSIONS_MOVEIT_EXTENSIONS_H
#define MOVEIT_EXTENSIONS_MOVEIT_EXTENSIONS_H

#include <moveit/distance_field/propagation_distance_field.h>

namespace moveit_extensions {
  class InterpolatedPropagationDistanceField : public distance_field::PropagationDistanceField {
  public:
    using PropagationDistanceField::PropagationDistanceField; // 継承コンストラクタ

    // cellはresolution間隔で並んでいる. デフォルトではxyzに対応するcellの値をそのまま返すので、cellの境界で値が不連続で変化し、また、cellの中では値が変わらない. これは、勾配を利用した干渉回避には不適切である.
    // cellの値はcellの中心の値であるとし、周囲8cellの中心の値を線形補間して返す.
    // getDistance()関数はPropagationDistanceFieldクラス中の各所で使われており、高速性が重要であるため、overrideしない.
    double getInterpolatedDistance(double x, double y, double z) const;
  };
};

#endif
