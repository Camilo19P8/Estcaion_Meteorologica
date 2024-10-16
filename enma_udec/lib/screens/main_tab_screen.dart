import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'main_tab_screen.dart';
import 'calendar_screen.dart';
import 'data_screen.dart';
import 'profile_screen.dart';
import 'alerts_screen.dart';
import 'contact_screen.dart';

class MainTabScreen extends StatefulWidget {
  @override
  _MainTabScreenState createState() => _MainTabScreenState();
}

class _MainTabScreenState extends State<MainTabScreen> {
  int _currentIndex = 0;
  final Color _bottomNavBarColor = const Color.fromRGBO(18, 17, 24, 100);
  final List<Widget> _children = [
    CalendarScreen(), // Reemplaza Home por Calendario
    DataScreen(),
    ProfileScreen(),
    AlertsScreen(),
    ContactScreen(),
  ];

  void onTabTapped(int index) {
    setState(() {
      _currentIndex = index;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('ENMA'),
        centerTitle: true,
        backgroundColor: _bottomNavBarColor,
      ),
      body: _children[_currentIndex],
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: _currentIndex,
        onTap: onTabTapped,
        backgroundColor: _bottomNavBarColor,
        selectedItemColor: Colors.tealAccent[400],
        unselectedItemColor: Colors.white70,
        items: const [
          BottomNavigationBarItem(
            icon: Icon(Icons.calendar_today),
            label: 'Calendario',
          ),
          BottomNavigationBarItem(
            icon: FaIcon(FontAwesomeIcons.cloudSun),
            label: 'Datos',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.person),
            label: 'Perfil',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.notifications),
            label: 'Alertas',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.contact_mail),
            label: 'Contacto',
          ),
        ],
      ),
    );
  }
}
