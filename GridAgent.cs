#define USE_VECTOR_3_GRID_AGENT
using System;
using System.Numerics;
using Numerics;

namespace Grids
{
#if USE_DOUBLE_3_GRID_AGENT
    using GridAgentVector3 = Double3;
#endif
#if USE_VECTOR_3_GRID_AGENT
    using GridAgentVector3 = Vector3;
#endif
    public class GridAgent : IGridAgent
    {
        private Int3 _forward;
        private Func<Vector3, Int3> _inverseTransform;
        private Func<Int3, bool> _isWalkable;
        private float _speed;
        private Func<Int3, Vector3> _transform;

        public Int3 Forward
        {
            set => _forward = value;
        }

        public Func<Vector3, Int3> InverseTransform
        {
            set => _inverseTransform = value;
        }

        public Func<Int3, bool> IsWalkable
        {
            get => _isWalkable;
            set => _isWalkable = value;
        }

        public float Speed
        {
            set => _speed = value;
        }

        public Func<Int3, Vector3> Transform
        {
            set => _transform = value;
        }

        public bool CanMove(Vector3 position)
        {
            var inversePosition = _inverseTransform(position);
            if (IsWalkable(inversePosition + _forward)) return true;
            var centerDelta = _transform(inversePosition) - position;
            return centerDelta.LengthSquared() > 0;
        }

        private static bool ChangeDirectionMoveToCenter(
            Int3 absForward,
            Vector3 center,
            Int2 normalizedVelocity,
            ref GridAgentVector3 position
        )
        {
            if (absForward.X > 0)
            {
                var delta = center.Y - position.Z;
                var absDelta = Math.Abs(delta);
                if (absDelta > float.Epsilon)
                {
                    if (absDelta < absForward.X)
                    {
                        position.Z = center.Y;
                        position.X += normalizedVelocity.X * (absForward.X - absDelta);
                    }
                    else
                    {
                        position.Z += absForward.X * delta.Normalize();
                    }

                    return true;
                }
            }
            else if (absForward.Y > 0)
            {
                var delta = center.X - position.X;
                var absDelta = Math.Abs(delta);
                if (absDelta > float.Epsilon)
                {
                    if (absDelta < absForward.Y)
                    {
                        position.X = center.X;
                        position.Z = normalizedVelocity.Y * (absForward.Y - absDelta);
                    }
                    else
                    {
                        position.X += absForward.Y * delta.Normalize();
                    }

                    return true;
                }
            }

            return false;
        }
#if USE_DOUBLE_3_GRID_AGENT
        private static double GetDelta(TimeSpan dt)
        {
            return dt.TotalSeconds;
        }
#endif
#if USE_VECTOR_3_GRID_AGENT
        private static float GetDelta(TimeSpan dt)
        {
            return (float)dt.TotalSeconds;
        }
#endif

        public Int3 GetNextCell(Int3 direction, Vector3 position)
        {
            return _inverseTransform(position) + direction;
        }

        public bool MoveToCenterStep(TimeSpan dt, ref GridAgentVector3 position)
        {
            if (_speed == 0) return false;
            var velocity = _forward * _speed * GetDelta(dt);
            var absForward = _forward.Abs();
            var normalizedVelocity = new Int2
            {
                X = velocity.X.Normalize(),
                Y = velocity.Y.Normalize()
            };
            var inversePosition = _inverseTransform(position);
            var center = _transform(inversePosition);
            var neighbor = inversePosition + _forward;
            if (IsWalkable(neighbor))
            {
                if (!ChangeDirectionMoveToCenter(absForward, center, normalizedVelocity, ref position))
                {
                    position.X += velocity.X;
                    position.Z += velocity.Y;
                }

                return true;
            }

            return NotWalkableMoveToCenter(absForward, center, ref position);
        }

        private static bool NotWalkableMoveToCenter(Int3 absForward, Vector3 center, ref GridAgentVector3 position)
        {
            if (absForward.X > 0)
            {
                var delta = center.X - position.X;
                var absDelta = Math.Abs(delta);
                if (absDelta > float.Epsilon)
                {
                    position.X = absDelta < absForward.X
                        ? center.X
                        : position.X + absForward.X * delta.Normalize();
                    return true;
                }

                return false;
            }

            if (absForward.Y > 0)
            {
                var delta = center.Y - position.Z;
                var absDelta = Math.Abs(delta);
                if (absDelta > float.Epsilon)
                {
                    position.Z = absDelta < absForward.Y
                        ? center.Y
                        : position.Z + absForward.Y * delta.Normalize();
                    return true;
                }

                return false;
            }

            throw new Exception();
        }
#if USE_DOUBLE_3_GRID_AGENT
        private static double SqrMagnitude(GridAgentVector3 value)
        {
            return value.SqrMagnitude();
        }
#endif
#if USE_VECTOR_3_GRID_AGENT
        private static double SqrMagnitude(GridAgentVector3 value)
        {
            return value.LengthSquared();
        }
#endif
        public bool Step(TimeSpan dt, ref GridAgentVector3 position)
        {
            var inversePosition = _inverseTransform(position);
            var delta = _forward * _speed * GetDelta(dt);
            if (IsWalkable(inversePosition + _forward))
            {
                position += delta;
                return true;
            }

            var centerPosition = _transform(inversePosition);
            var centerDelta = centerPosition - position;
            var sqrCenterDelta = SqrMagnitude(centerDelta);
            if (sqrCenterDelta == 0) return false;
            position = SqrMagnitude(centerDelta) > SqrMagnitude(delta) ? position + delta : centerPosition;
            return true;
        }
    }
}