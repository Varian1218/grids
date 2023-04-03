using System;

namespace Grids
{
    public static class GridUtils
    {
        public static (int, int) Select<T>(IGrid grid, Func<int, T> selector, ref T[,] values)
        {
            for (var i = 0; i < grid.Width; i++)
            {
                for (var j = 0; j < grid.Height; j++)
                {
                    values[i, j] = selector(grid[i, j]);
                }
            }

            return (grid.Height, grid.Width);
        }
    }
}