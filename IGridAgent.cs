using System;
using System.Numerics;
using Numerics;

namespace Grids
{
    public interface IGridAgent
    {
        Func<Int3, bool> IsWalkable { get; set; }
        bool MoveToCenterStep(TimeSpan dt, ref Vector3 position);
        bool Step(TimeSpan dt, ref Vector3 position);
    }
}