namespace Grids
{
    public class Grid : IGrid
    {
        private int[,] _cells;

        public int this[int x, int y]
        {
            get => _cells[x, y];
            set => _cells[x, y] = value;
        }

        public int Height { get; set; }

        public (int Height, int Width) Size
        {
            set => _cells = new int[value.Width, value.Height];
        }

        public int Width { get; set; }
    }
}