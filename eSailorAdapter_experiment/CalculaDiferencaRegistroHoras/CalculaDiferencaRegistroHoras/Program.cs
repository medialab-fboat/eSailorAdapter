using System.Globalization;

//PROGRAMA QUE LIMPA DADOS ENVIADOS
namespace CalculaDiferencaRegistroHoras
{
    internal class Program
    {
        static void Main_()
        {
            string inputFilePath = "C:\\experimento\\raw_input.txt"; // Caminho do arquivo de entrada
            string outputFilePath = "C:\\experimento\\output.txt"; // Caminho do arquivo de saída


            // Abrir o arquivo de entrada para leitura
            using (StreamReader sr = new StreamReader(inputFilePath))
            {
                // Abrir o arquivo de saída para escrita
                using (StreamWriter sw = new StreamWriter(outputFilePath))
                {
                    string line;
                    int observedValue = int.MinValue;
                    while ((line = sr.ReadLine()) != null)
                    {                        
                        string[] lineContent = line.Split(' ');
                        
                        if (Convert.ToInt32(lineContent[0]) != observedValue) 
                        {
                            sw.WriteLine($"{lineContent[0]};{lineContent[1]}");
                            observedValue = Convert.ToInt32(lineContent[0]);
                        }
                    }
                }
            }
        }
    }
}