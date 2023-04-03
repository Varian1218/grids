namespace Grids
{
    public interface IGrid
    {
        int this[int x, int y] { get; set; }
        int Height { get; set; }
        (int Height, int Width) Size { set; }
        int Width { get; set; }
    }
}