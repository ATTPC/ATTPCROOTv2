#include "TH2F.h"
#include "TF2.h"
#include "TCanvas.h"
#include "TMarker.h"
#include "TFitResultPtr.h"
#include "TMath.h"
#include <vector>
#include <iostream>

// 定义交点结构体（存储x,y坐标）
struct IntersectionPoint {
    double x;
    double y;
    IntersectionPoint(double x_, double y_) : x(x_), y(y_) {}
};


std::vector<IntersectionPoint> fitTH2FAndFindIntersections(
    TGraph* g1, 
    TGraph* g2, 
    TH2F * h2,
    double xMin, double xMax, 
    double yMin, double yMax
) {
    // 输出容器：存储交点
    std::vector<IntersectionPoint> intersections;

    std::cout << "筛选完成数量=" << g1->GetN()<<std::endl; 
    std::cout << "筛选完成数量=" << g2->GetN()<<std::endl; 
    // 检查输入直方图有效性
    if (!g2) {
        std::cerr << "错误：输入的TH2F为空！" << std::endl;
        return intersections;
    }
    if (!g1) {
        std::cerr << "错误：输入的TH2F为空！" << std::endl;
        return intersections;
    }

    // ==============================================
    // 1. 定义拟合函数：二次函数和指数函数（y关于x的函数）
    // ==============================================
    // 二次函数：f(x,y) = y - (a*x² + b*x + c) → 拟合目标f=0即y = a x² + b x + c
    TF1* f_quad = new TF1("f_quad", "[0]*x*x + [1]*x + [2]", xMin, xMax);
    TF1* f_exp = new TF1("f_exp", "[0]*exp([1]*x)+[2]", xMin, xMax);

    // ==============================================
    // 2. 设置初始参数（关键：影响拟合收敛性）
    // ==============================================
    // 二次函数初始参数：根据数据大致趋势猜测（振幅、线性项、常数项）
    f_quad->SetParameters(-0.0005, 0.64, -93);  // a=0.5, b=0, c=0
    f_exp->SetParameters(-3000, -0.001, 1000);  // a=0.5, b=0, c=0



    // 拟合二次函数
    TFitResultPtr fit_quad = g1->Fit(f_quad, "QRS+");  // "Q"安静模式, "S"保存结果, "+"叠加绘制
    TFitResultPtr fit_exp = g2->Fit(f_exp, "QRS+");  // "Q"安静模式, "S"保存结果, "+"叠加绘制
    if (!fit_quad->IsValid()) {
        std::cerr << "警告：二次函数拟合失败！" << std::endl;
        return intersections;
    }
    if (!fit_exp->IsValid()) {
        std::cerr << "警告：二次函数拟合失败！" << std::endl;
        return intersections;
    }
    h2->Draw("cloz"); g1->Draw("same");g2->Draw("same");


    // ==============================================
    // 4. 提取拟合参数
    // ==============================================
    double a = f_quad->GetParameter(0);  // 二次函数：a x² + b x + c
    double b = f_quad->GetParameter(1);
    double c = f_quad->GetParameter(2);

    double d = f_exp->GetParameter(0);   // 指数函数：d exp(e x) + f
    double e = f_exp->GetParameter(1);
    double f = f_exp->GetParameter(2);

    std::cout << "\n拟合参数：" << std::endl;
    std::cout << "二次函数：y = " << a << "x² + " << b << "x + " << c << std::endl;
    std::cout << "指数函数：y = " << d << "exp(" << e << "x) + " << f << std::endl;

    // ==============================================
    // 5. 数值求解交点：解方程 a x² + b x + c = d exp(e x) + f
    // ==============================================
    // 定义差值函数：h(x) = 二次函数值 - 指数函数值 → 交点处h(x)=0
    auto h = [&](double x) {
        double y_quad = a*x*x + b*x + c;    // 二次函数在x处的y值
        double y_exp = d*TMath::Exp(e*x) + f;  // 指数函数在x处的y值
        //double y_exp = d*x*x + e*x + f;  // 指数函数在x处的y值
        return y_quad - y_exp;  // 差值（接近0时为交点）
    };

    // 网格搜索找交点（在x范围内密集采样）
    const int nSteps = 10000;  // 采样步数（越大精度越高）
    double step = (xMax - xMin) / nSteps;
    double eps = 1e-3;         // 判定阈值（差值绝对值小于此值视为交点）
    double prevSign = 0;       // 记录上一步的符号（用于检测符号变化，即穿过0点）

    for (int i = 0; i < nSteps; ++i) {
        double x = xMin + i * step;
        double val = h(x);
        double sign = TMath::Sign(1.0, val);  // 获取符号（1或-1）

        // 检测符号变化（说明穿过0点，存在交点）或直接找到接近0的点
        if ((sign != prevSign && prevSign != 0) || TMath::Abs(val) < eps) {
            // 线性插值精确化交点x（提高精度）
            double x1 = x - step;
            double x2 = x;
            double y1 = h(x1);
            double y2 = val;
            double x_intersect = x1 - (y1 * (x2 - x1)) / (y2 - y1);  // 线性插值求根
            double y_intersect = a*x_intersect*x_intersect + b*x_intersect + c;  // 代入二次函数求y

            // 保存交点
            intersections.emplace_back(x_intersect, y_intersect);
            std::cout << "找到交点：(x=" << x_intersect << ", y=" << y_intersect << ")" << std::endl;

            // 在画布上标记交点（红色星号）
            TMarker* m = new TMarker(x_intersect, y_intersect, 29);  // 29=星号
            m->SetMarkerColor(kRed);
            m->SetMarkerSize(2);
            m->Draw("same");
        }

        if (val != 0) prevSign = sign;  // 更新符号（跳过0值，避免重复判定）
    }

    if (intersections.empty()) {
        std::cout << "未找到交点！" << std::endl;
    }

    // 美化画布并保存

    return intersections;
}

// 函数：从原始TGraph中筛选出X∈[xmin,xmax]且Y∈[ymin,ymax]的点，返回新TGraph
TGraph* filterGraphByXY(TGraph* gr, double xmin, double xmax, double ymin, double ymax) {
    if (!gr) {
        std::cerr << "错误：输入TGraph为空！" << std::endl;
        return nullptr;
    }

    TGraph* grFiltered = new TGraph();  // 存储筛选后的点
    grFiltered->SetName("gr_filtered");
    grFiltered->SetTitle("筛选后的点（X和Y范围限制）");

    // 遍历原始TGraph的所有点
    for (int i = 0; i < gr->GetN(); ++i) {
        double x, y;
        gr->GetPoint(i, x, y);  // 获取第i个点的(x,y)

        // 检查是否同时在X和Y范围内
        if (x >= xmin && x <= xmax && y >= ymin && y <= ymax) {
            grFiltered->SetPoint(grFiltered->GetN(), x, y);  // 添加到新图
        }
    }

    std::cout << "筛选完成：原始点数量=" << gr->GetN() 
              << "，筛选后点数量=" << grFiltered->GetN() << std::endl;
    return grFiltered;
}

