using System;
using System.Numerics;
using Floats;
using Ints;
using Transforms;

namespace Grids
{
    public class GridAgent : IGridAgent
    {
        private Func<Int2, (float, float)> _getCenter;
        private int _height;
        private Func<float, float, Int2> _inverseTransform;
        private bool[,] _locked;
        private ITransform _transform;
        private Int2 _velocity;
        private int _width;

        public Func<Int2, (float, float)> GetCenter
        {
            set => _getCenter = value;
        }

        public Func<float, float, Int2> InverseTransform
        {
            set => _inverseTransform = value;
        }

        public (int X, int Y) Size
        {
            set
            {
                _height = value.Y;
                _locked = new bool[value.X, value.Y];
                _width = value.X;
            }
        }

        public ITransform Transform
        {
            set => _transform = value;
        }

        public Int2 Velocity
        {
            set => _velocity = value;
        }

        private static bool ChangeDirectionMoveToCenter(
            (float X, float Y) absVelocity,
            (float X, float Y) center,
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

        private bool IsWalkable(int x, int y)
        {
            if (x < 0 || y < 0 || x >= _width || y >= _height) return false;
            return !_locked[x, y];
        }

        public void Lock(bool value, int x, int y)
        {
            _locked[x, y] = value;
        }

        private static void NotWalkableMoveToCenter(
            (float X, float Y) absVelocity,
            (float X, float Y) center,
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
                    return;
                }
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
                }
            }
        }

        public void Step(TimeSpan dt)
        {
            if (_velocity.SqrMagnitude() == 0) return;
            var velocity = _velocity * (float)dt.TotalSeconds;
            (float X, float Y) absVelocity = (Math.Abs(velocity.X), Math.Abs(velocity.Y));
            var normalizedVelocity = new Int2
            {
                X = velocity.X.Normalize(),
                Y = velocity.Y.Normalize()
            };
            var position = _transform.Position;
            var inversePosition = _inverseTransform(position.X, position.Z); 
            (float X, float Y) center = _getCenter(inversePosition);
            var neighbor = inversePosition + normalizedVelocity;
            if (IsWalkable(neighbor.X, neighbor.Y))
            {
                if (!ChangeDirectionMoveToCenter(absVelocity, center, normalizedVelocity, ref position))
                {
                    position.X += velocity.X;
                    position.Z += velocity.Y;
                }
            }
            else
            {
                NotWalkableMoveToCenter(absVelocity, center, ref position);
            }

            _transform.Position = position;
        }

        public void SetVelocity(Vector2 value)
        {
            Velocity = new Int2
            {
                X = (int)value.X,
                Y = (int)value.Y
            };
        }
    }
}