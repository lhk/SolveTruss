using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SolveTruss
{
    class Program
    {
        static void Main(string[] args)
        {
            Matrix test = new Matrix(4, 3);
            test.m[0, 0] = 0; test.m[1, 0] = 1; test.m[2, 0] = 2; test.m[3, 0] = 2;
            test.m[0, 1] = 1; test.m[1, 1] = 1; test.m[2, 1] = 1; test.m[3, 1] = 2;
            test.m[0, 2] = 1; test.m[1, 2] = 1; test.m[2, 2] = 2; test.m[3, 2] = 2;

            test.print();
            Console.Out.WriteLine("-----------------------------------------");
            test.gauss();
            Console.Out.WriteLine("-----------------------------------------");
            test.print();

            Console.In.Read();
        }
    }
}
