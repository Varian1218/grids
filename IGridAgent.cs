using System;
using System.Numerics;

namespace Grids
{
    public interface IGridAgent
    {
        void Lock(bool value, int x, int y);
        bool MoveToCenterStep(TimeSpan dt, ref Vector3 position);
        bool Step(TimeSpan dt, ref Vector3 position);
    }
}