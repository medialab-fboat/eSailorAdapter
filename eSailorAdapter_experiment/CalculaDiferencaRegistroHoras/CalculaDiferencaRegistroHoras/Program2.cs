using System.Globalization;

//PROGRAMA QUE CALCULA DELAY
namespace CalculaDiferencaRegistroHoras
{
    internal class Program2
    {
        static void Main()
        {
            string inputFilePath = "C:\\experimento\\delay_calc_input2.txt"; // Caminho do arquivo de entrada
            string outputFilePath = "C:\\experimento\\delay_calc_output2.txt"; // Caminho do arquivo de saída

            // Abrir o arquivo de entrada para leitura
            using (StreamReader sr = new StreamReader(inputFilePath))
            {
                // Abrir o arquivo de saída para escrita
                using (StreamWriter sw = new StreamWriter(outputFilePath))
                {
                    string line;
                    while ((line = sr.ReadLine()) != null)
                    {
                        // Assumindo que as horas estão separadas por um espaço
                        string[] times = line.Split(' ');

                        if (times.Length >= 2)
                        {
                            // Converter as horas para TimeSpan
                            TimeSpan time1 = TimeSpan.ParseExact(times[0], @"hh\:mm\:ss\:fff", CultureInfo.InvariantCulture);
                            TimeSpan time2 = TimeSpan.ParseExact(times[1], @"hh\:mm\:ss\:fff", CultureInfo.InvariantCulture);

                            // Calcular a diferença entre a segunda hora e a primeira
                            TimeSpan difference = time2 - time1;

                            // Escrever a linha original seguida pela diferença no arquivo de saída
                            sw.WriteLine($"{line} {difference:hh\\:mm\\:ss\\:fff}");
                        }
                    }
                }
            }

            Console.WriteLine("Diferenças calculadas e escritas no arquivo output.txt");
        }
    }
}