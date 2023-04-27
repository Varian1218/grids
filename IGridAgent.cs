using System;
using System.Numerics;
using Numerics;

namespace Grids
{
    public interface IGridAgent
    {
        Int3 Forward { get; set; }
        Func<Int3, bool> IsWalkable { get; set; }
        bool MoveToCenterStep(float delta, ref Vector3 forward, ref Vector3 position);
        bool Step(float delta, ref Vector3 position);
    }
}