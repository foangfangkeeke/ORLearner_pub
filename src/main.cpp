#include "cutting_stock_problem.hpp"

int main()
{
    auto pb = ColumnGeneration(CUTTINGSTOCK);
    pb.Run();

    return 0;
}