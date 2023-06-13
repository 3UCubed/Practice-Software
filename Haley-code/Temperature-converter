/*
 * Program to convert temperatures between Celsius, Fahrenheit, and Kelvin
 * Accepts user input for the starting temperature and desired conversion
 */

#include <iostream>
using namespace std;


//Celsius to Kelvin
double celsiustoKelvin(double celsius) {
    return celsius + 273.15;
}

//Celsius to Fahrenheit
double celsiustoFahrenheit(double celsius) {
    return (celsius*(9/5)) + 32;
}

//Kelvin to Celsius
double kelvintoCelsius(double kelvin) {
    return kelvin - 273.15;
}

//Kelvin to Fahrenheit
double kelvintoFahrenheit(double kelvin) {
    double celsius = kelvintoCelsius(kelvin);
    return celsiustoFahrenheit(celsius);
}

//Fahrenheit to Celcius
double fahrenheittoCelsius(double fahrenheit) {
    return (fahrenheit - 32) * 5/9;
}

//Fahrenheit to Kelvin
double fahrenheittoKelvin(double fahrenheit) {
    return (fahrenheit - 32) * 5/9 + 273.15;
}






int main() {
    double temperature;
    int choice;

    cout << "This is the temperature converter. I will give you a list of options. Please choose one when prompted. " << endl;
    cout << "Option 1: Celsius to Kelvin" << endl;
    cout << "Option 2: Celsius to Fahrenheit" << endl;
    cout << "Option 3: Kelvin to Celsius" << endl;
    cout << "Option 4: Kelvin to Fahrenheit" << endl;
    cout << "Option 5: Fahrenheit to Celsius" << endl;
    cout << "Option 6: Fahrenheit to Kelvin" << endl;
    cout << "Enter your choice:" << endl;
    cin >> choice;


    cout << "Enter a temperature to convert:";
    cin >> temperature;

    double convertedTemperature;

    //Switch statements to implement the calculations
    switch(choice) {
        case 1:
            convertedTemperature = celsiustoKelvin(temperature);
            cout << "Converted temperature: " << convertedTemperature << " °K" << endl;
            break;
        case 2:
            convertedTemperature = celsiustoFahrenheit(temperature);
            cout << "Converted temperature: " << convertedTemperature << " °F" << endl;
            break;
            //todo
        case 3:
            convertedTemperature = kelvintoCelsius(temperature);
            cout << "Converted temperature: " << convertedTemperature << " °C" << endl;
            break;
        case 4:
            convertedTemperature = kelvintoFahrenheit(temperature);
            cout << "Converted temperature: " << convertedTemperature << " °F" << endl;
            break;
        case 5:
            convertedTemperature = fahrenheittoCelsius(temperature);
            cout << "Converted temperature: " << convertedTemperature << " °C" << endl;
            break;
        case 6:
            convertedTemperature = fahrenheittoKelvin(temperature);
            cout << "Converted temperature: " << convertedTemperature << " °K" << endl;
            break;
        default:
            cout << "Invalid choice." << endl;
            break;
    }
    return 0;
}
