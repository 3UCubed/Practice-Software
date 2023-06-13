# TLE-Code
#This code takes in a TLE file then parses out the first line. After this, it outputs the TLE time in Julian Day Fraction, and the TLE
time is years, days, hours, minutes, and seconds.

/*
Haley Joerger
CS 397 - Internship
EdEon Department
*/



#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <math.h>
#include <algorithm>
using namespace std;


//Struct in order to store all of the derived types
struct tleTime {
    int years;
    double fdays;
    double days, hours, minutes, seconds;
    double fract_part;
};


int main(void) {
    int lineIndice = 9;
    int LineOne = 0;
    vector<string> value(lineIndice);
    ifstream file;

    file.open("sanjsat.txt", ios::in); // Txt file changed from C: to the actual file name
    if (file.is_open()) // Opening up the file
    {
        for (int i = 0; i < lineIndice; i++) {
            file >> value[i];
        }
        file.close();
    }
    else {
        cout << "Could not access file."; //Error message for if file does not output
        return -1;
    }

    vector <string>::iterator itr;

    
    for (int i = 0; i < lineIndice; i++) //Outputs the first line of the TLE file
    {
        cout << value[i] << " "; 
    }


    cout << "\nData to parse: " << value[3] << " ";  //Parces out the 3rd element, the element that needs to be parsed.
    cout << "\n" << endl;
    cout << "Parsed value:";

   
    string index = value[3]; //Putting into index, that wat we can parse it
    stringstream str;
 
    
 
    //Adding value to variables
    int years = stoi(index.substr(0, 2)) + 2000; //Setting the years. Using the substr is a hack, additionally using the stoi is a hack when working with integers
    double fdays = stod(index.substr(2)); //Setting the fraction days, using the stod is a hack when working with doubles.
    double days;
    double hours;
    double minutes;
    double seconds; //Creating variables for the rest
    double frac_part = fdays; //Creating distinction between them
    frac_part = modf(frac_part, &days); //Using modf to perform math when a decimal point is present

    //Performing mathematical operations
    frac_part *= 24; // 24 hours in 1 day
    frac_part = modf(frac_part, &hours);
    frac_part *= 60; // 60 minutes in 1 hour
    frac_part = modf(frac_part, &minutes);
    frac_part *= 60; // 60 seconds in 1 minute
    frac_part = modf(frac_part, &seconds);
                

    cout << "\nTLE time in Julian Day Fraction: " << years << " years, "; //outputting the Julian years and the days
    cout << fixed << setprecision(8) << fdays << " days."; //Outputting the days with the decimal places
    cout << fixed << setprecision(0); //Changing the precision back to zero, that way there's not extra zeros
    cout << "\nTLE time: " << years << " years, " << days << " days, "; //Spitting out the exact data that we need to know
    cout << hours << " hours, " << minutes << " minutes, " << seconds << " seconds." << endl; 



    if (value[3] != ".") {
        double julianFractionDay = index[6];
    }
}



