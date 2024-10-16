import 'package:flutter/material.dart';

class AlertsScreen extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Expanded(
          flex: 2,
          child: Container(
            color: Colors.red[300],
            child: const Center(
              child: Text(
                'Alerta Principal: Altas temperaturas',
                style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
              ),
            ),
          ),
        ),
        Expanded(
          flex: 3,
          child: ListView(
            children: const [
              ListTile(
                title: Text('Alerta de altas temperaturas'),
                subtitle: Text('19 Abr, 21:00'),
              ),
              ListTile(
                title: Text('Alerta de Precipitación'),
                subtitle: Text('24 Abr - 25 Abr'),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
