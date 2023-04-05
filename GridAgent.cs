using System;
using System.Numerics;
using Numerics;

namespace Grids
{
    public class GridAgent : IGridAgent
    {
        private Func<Vector3, Int3> _inverseTransform;
        private Func<Int3, bool> _isWalkable;
        private Func<Int3, Vector3> _transform;

        public Func<Vector3, Int3> InverseTransform
        {
            set => _inverseTransform = value;
        }

        public Func<Int3, bool> IsWalkable
        {
            get => _isWalkable;
            set => _isWalkable = value;
        }

        public Func<Int3, Vector3> Transform
        {
            set => _transform = value;
        }

        public Int3 Velocity { get; set; }

        public bool CanMove(Vector3 position)
        {
            var inversePosition = _inverseTransform(position);
            if (IsWalkable(inversePosition + Velocity)) return true;
            var centerDelta = _transform(inversePosition) - position;
            return centerDelta.LengthSquared() > 0;
        }

        private static bool ChangeDirectionMoveToCenter(
            (float X, float Y) absVelocity,
            Vector3 center,
            Int2 normalizedVelocity,
            ref Vector3 position
        )
        {
            if (absVelocity.X > 0)
            {
                var delta = center.Y - position.Z;
                var absDelta = Math.Abs(delta);
                if (absDelta > float.Epsilon)
                {
                    if (absDelta < absVelocity.X)
                    {
                        position.Z = center.Y;
                        position.X += normalizedVelocity.X * (absVelocity.X - absDelta);
                    }
                    else
                    {
                        position.Z += absVelocity.X * delta.Normalize();
                    }

                    return true;
                }
            }
            else if (absVelocity.Y > 0)
            {
                var delta = center.X - position.X;
                var absDelta = Math.Abs(delta);
                if (absDelta > float.Epsilon)
                {
                    if (absDelta < absVelocity.Y)
                    {
                        position.X = center.X;
                        position.Z = normalizedVelocity.Y * (absVelocity.Y - absDelta);
                    }
                    else
                    {
                        position.X += absVelocity.Y * delta.Normalize();
                    }

                    return true;
                }
            }

            return false;
        }

        public Int3 GetNextCell(Int3 direction, Vector3 position)
        {
            return _inverseTransform(position) + direction;
        }

        public bool MoveToCenterStep(TimeSpan dt, ref Vector3 position)
        {
            if (Velocity.SqrMagnitude() == 0) return false;
            var velocity = Velocity * (float)dt.TotalSeconds;
            (float X, float Y) absVelocity = (Math.Abs(velocity.X), Math.Abs(velocity.Y));
            var normalizedVelocity = new Int2
            {
                X = velocity.X.Normalize(),
                Y = velocity.Y.Normalize()
            };
            var inversePosition = _inverseTransform(position);
            var center = _transform(inversePosition);
            var neighbor = inversePosition + Velocity;
            if (IsWalkable(neighbor))
            {
                if (!ChangeDirectionMoveToCenter(absVelocity, center, normalizedVelocity, ref position))
                {
                    position.X += velocity.X;
                    position.Z += velocity.Y;
                }

                return true;
            }

            return NotWalkableMoveToCenter(absVelocity, center, ref position);
        }

        private static bool NotWalkableMoveToCenter(
            (float X, float Y) absVelocity,
            Vector3 center,
            ref Vector3 position
        )
        {
            if (absVelocity.X > 0)
            {
                var delta = center.X - position.X;
                var absDelta = Math.Abs(delta);
                if (absDelta > float.Epsilon)
                {
                    position.X = absDelta < absVelocity.X
                        ? center.X
                        : position.X + absVelocity.X * delta.Normalize();
                    return true;
                }

                return false;
            }

            if (absVelocity.Y > 0)
            {
                var delta = center.Y - position.Z;
                var absDelta = Math.Abs(delta);
                if (absDelta > float.Epsilon)
                {
                    position.Z = absDelta < absVelocity.Y
                        ? center.Y
                        : position.Z + absVelocity.Y * delta.Normalize();
                    return true;
                }

                return false;
            }

            throw new Exception();
        }

        public bool Step(TimeSpan dt, ref Vector3 position)
        {
            var inversePosition = _inverseTransform(position);
            var delta = Velocity * dt.TotalSeconds;
            if (IsWalkable(inversePosition + Velocity))
            {
                position += delta;
                return true;
            }

            var centerPosition = _transform(inversePosition);
            var centerDelta = centerPosition - position;
            var sqrCenterDelta = centerDelta.LengthSquared();
            if (sqrCenterDelta == 0) return false;
            position = centerDelta.LengthSquared() > delta.SqrMagnitude() ? position + delta : centerPosition;
            return true;
        }

        public void SetVelocity(Vector2 value)
        {
            Velocity = new Int3
            {
                X = (int)value.X,
                Y = (int)value.Y
            };
        }
    }
}