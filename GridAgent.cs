using System;
using System.Numerics;
using Numerics;

namespace Grids
{
    public class GridAgent : IGridAgent
    {
        private int _height;
        private Func<Vector3, Int3> _inverseTransform;
        private bool[,] _locked;
        private Func<Int3, Vector3> _transform;
        private int _width;

        public Func<Vector3, Int3> InverseTransform
        {
            set => _inverseTransform = value;
        }

        public Int2 Size
        {
            set
            {
                _height = value.Y;
                _locked = new bool[value.X, value.Y];
                _width = value.X;
            }
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

        public bool IsWalkable(int x, int y)
        {
            if (x < 0 || y < 0 || x >= _width || y >= _height) return false;
            return !_locked[x, y];
        }

        public bool IsWalkable(Int3 position)
        {
            return IsWalkable(position.X, position.Y);
        }

        public void Lock(bool value, int x, int y)
        {
            _locked[x, y] = value;
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
            if (IsWalkable(neighbor.X, neighbor.Y))
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

        // public bool Step(TimeSpan dt, Vector3 position, out Double3 remainingDelta)
        // {
        //     var inversePosition = _inverseTransform(position);
        //     delta = Velocity * dt.TotalSeconds;
        //     if (IsWalkable(inversePosition + Velocity))
        //     {
        //         delta = Velocity * dt.TotalSeconds;
        //         remainingDelta = Double3.Zero;
        //         return true;
        //     }
        //
        //     var center = _getCenter(inversePosition);
        //     var centerDelta = center - position;
        //     var centerSqrDelta = centerDelta.LengthSquared(); 
        //     if (centerSqrDelta > 0)
        //     {
        //         if (centerSqrDelta > delta.SqrMagnitude())
        //         {
        //             delta = Velocity * dt.TotalSeconds;
        //             remainingDelta = Double3.Zero;
        //             return true;
        //         }
        //
        //         remainingDelta = delta - centerDelta;
        //         delta = centerDelta;
        //         return true;
        //     }
        //
        //     delta = Double3.Zero;
        //     remainingDelta = Double3.Zero;
        //     return false;
        // }

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