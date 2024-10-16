import 'package:flutter/material.dart';
import 'screens/login_screen.dart';

void main() {
  runApp(WeatherMonitoringApp());
}

class WeatherMonitoringApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Weather Monitoring',
      theme: ThemeData(
        brightness: Brightness.dark,
        primaryColor: Colors.blueGrey[800],
        scaffoldBackgroundColor: Colors.grey[900],
        buttonTheme: ButtonThemeData(
          buttonColor: Colors.tealAccent[400],
        ),
        textTheme: const TextTheme(
          bodyLarge: TextStyle(color: Colors.white70),
          bodyMedium: TextStyle(color: Colors.white),
        ),
      ),
      debugShowCheckedModeBanner: false,
      home: LoginScreen(),
    );
  }
}
