using System;

namespace Grids
{
    public interface IGridAgent
    {
        void Lock(bool value, int x, int y);
        void Step(TimeSpan dt);
    }
}