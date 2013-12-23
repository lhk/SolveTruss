using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SolveTruss
{
    class Point
    {
        public double x;
        public double y;

        public Point(double x, double y){
            this.x=x;
            this.y=y;
        }
    }

    class Support : Point
    {
        public double force_x;
        public double force_y;

        public Support(double x, double y, double force_x, double force_y)
            : base(x, y)
        {
            this.force_x = force_x;
            this.force_y = force_y;
        }
    }

    class Node : Point
    {
        public Node(double x, double y) : base(x, y) { }
    }

    class Edge{
        public Point first;
        public Point second;
        public double tension;

        public Edge(Point first, Point second)
        {
            this.first = first;
            this.second = second;
        }
    }

    class Truss
    {
        public List<Support> supports;
        public List<Node> nodes;
        public List<Edge> edges;

        public Truss()
        {
            supports = new List<Support>();
            nodes = new List<Node>();
            edges = new List<Edge>();
        }

    }

    class Matrix
    {
        public double[,] m;
        public int cols;
        public int rows;
        public Matrix(int cols, int rows) {
            this.cols = cols;
            this.rows = rows;
            m = new double[cols,rows];
            for (int i = 0; i < cols; i++)
            {
                for (int j = 0; j < rows; j++)
                {
                    m[i,j] = 0;
                }
            }
        }

        public void mul(int line, double scale)
        {
            Console.Out.WriteLine("multiplying line: " + line + " with scale: " + scale);
            for (int i = 0; i < cols; i++)
            {
                m[i, line] *= scale;
            }
        }

        public void addlineto(int source, int dest, double scale)
        {
            Console.Out.WriteLine("adding line: " + source + " to line: " + dest + " multiple: " + scale);
            for (int i = 0; i < cols; i++)
            {
                m[i, dest] += m[i, source]*scale;
            }
        }

        public void swaplines(int source, int dest)
        {
            Console.Out.WriteLine("swapping lines: " + source + " and " + dest);
            for (int i = 0; i < cols; i++)
            {
                double temp = m[i, source];
                m[i, source] = m[i, dest];
                m[i, dest] = temp;
            }
        }

        public void print()
        {
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    Console.Out.Write(" " + Math.Floor(m[j,i])+"\t");
                }
                Console.Out.WriteLine();
            }
        }

        public void gauss() {
            int pivotrow = 0;
            for (int column = 0; column < cols; column++) {
                if (pivotrow >= rows) break;
                Console.Out.WriteLine("---------------------------------------");
                print();
                double elem = m[column, pivotrow];
                Console.Out.WriteLine("elem: "+elem+"at indices column: "+column+", pivotrow: "+pivotrow);
                if (elem != 0)
                {
                    mul(column, 1 / elem);
                    m[column, pivotrow] = 1;   // this should be 1 in any case, since we have multiplied its value elem with 1/elem.
                    for (int row = 0; row < pivotrow; row++)
                    {
                        Console.Out.WriteLine("before i: " + column + " k: " + row + " pivotj: " + pivotrow);
                        double scale = m[column, row];
                        addlineto(pivotrow,row,-scale);
                        m[column, row] = 0; // once again just to avoid floating point issues
                    }
                    for (int row = pivotrow + 1; row < rows; row++)
                    {
                        Console.Out.WriteLine("after i: " + column + " pivotj: " + pivotrow + " k: " + row);
                        double scale = m[column, row];
                        addlineto(pivotrow, row, -scale);
                        m[column, row] = 0;
                    }
                    pivotrow++;
                    continue;
                }
                else
                {
                    bool exists=false;
                    for (int row = pivotrow + 1; row < rows; row++)
                    {
                        if (m[column, row] != 0)
                        {
                            exists = true;
                            swaplines(pivotrow, row);
                            break;
                        }
                    }
                    if (exists)
                    {
                        elem = m[column, pivotrow];
                        Console.Out.WriteLine("elem: "+elem+" at indices column: "+column+", pivotrow: "+pivotrow);
                        mul(column, 1 / elem);
                        m[column, pivotrow] = 1;   // this should be 1 in any case, since we have multiplied its value elem with 1/elem.
                        for (int row = 0; row < pivotrow; row++)
                        {
                            Console.Out.WriteLine("before i: " + column + " k: " + row + " pivotj: " + pivotrow);
                            double scale = m[column, row];
                            addlineto(pivotrow,row,-scale);
                            m[column, row] = 0; // once again just to avoid floating point issues
                        }
                        for (int row = pivotrow + 1; row < rows; row++)
                        {
                            Console.Out.WriteLine("after i: " + column + " pivotj: " + pivotrow + " k: " + row);
                            double scale = m[column, row];
                            addlineto(pivotrow, row, -scale);
                            m[column, row] = 0;
                        }
                        pivotrow++;
                        continue;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
    }
}
