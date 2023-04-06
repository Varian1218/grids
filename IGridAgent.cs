#define USE_VECTOR_3_GRID_AGENT
using System;
#if USE_VECTOR_3_GRID_AGENT
using System.Numerics;
#endif
using Numerics;
using Times;

namespace Grids
{
#if USE_DOUBLE_3_GRID_AGENT
    using GridAgentVector3 = Double3;
#endif
#if USE_VECTOR_3_GRID_AGENT
    using GridAgentVector3 = Vector3;
#endif

    public interface IGridAgent
    {
        Func<Int3, bool> IsWalkable { get; set; }
        bool MoveToCenterStep(ShortTimeSpan dt, ref GridAgentVector3 position);
        bool Step(ShortTimeSpan dt, ref GridAgentVector3 position);
    }
}