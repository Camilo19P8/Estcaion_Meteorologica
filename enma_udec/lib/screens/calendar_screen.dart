import 'package:flutter/material.dart';
import 'package:table_calendar/table_calendar.dart';

class CalendarScreen extends StatefulWidget {
  @override
  _CalendarScreenState createState() => _CalendarScreenState();
}

class _CalendarScreenState extends State<CalendarScreen> {
  DateTime _selectedDay = DateTime.now();

  final Map<DateTime, List<String>> _climateRecords = {
    DateTime.utc(2024, 10, 14): ['Temperatura: 25°C', 'Humedad: 70%'],
    DateTime.utc(2024, 10, 15): ['Temperatura: 28°C', 'Humedad: 65%'],
    // Agrega más registros climáticos según el día
  };

  List<String> _getRecordsForDay(DateTime day) {
    return _climateRecords[day] ?? ['No hay registros para este día'];
  }

  @override
  Widget build(BuildContext context) {
    List<String> records = _getRecordsForDay(_selectedDay);

    return Padding(
      padding: const EdgeInsets.all(16.0),
      child: Column(
        children: [
          TableCalendar(
            focusedDay: _selectedDay,
            firstDay: DateTime.utc(2020, 10, 16),
            lastDay: DateTime.utc(2030, 3, 14),
            selectedDayPredicate: (day) => isSameDay(day, _selectedDay),
            onDaySelected: (selectedDay, focusedDay) {
              setState(() {
                _selectedDay = selectedDay;
              });
            },
          ),
          const SizedBox(height: 20),
          Text(
            'Registros climáticos para el ${_selectedDay.toLocal()}',
            style: const TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
          ),
          const SizedBox(height: 10),
          Expanded(
            child: ListView.builder(
              itemCount: records.length,
              itemBuilder: (context, index) {
                return ListTile(
                  leading: const Icon(Icons.cloud),
                  title: Text(records[index]),
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}
