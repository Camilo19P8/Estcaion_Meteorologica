import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';

class DataScreen extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: LayoutBuilder(
        builder: (context, constraints) {
          int crossAxisCount = constraints.maxWidth > 600 ? 4 : 2;

          return GridView.count(
            crossAxisCount: crossAxisCount, // Cambia entre 4 y 2 columnas
            crossAxisSpacing: 5.0, // Espacio horizontal
            mainAxisSpacing: 5.0, // Espacio vertical
            childAspectRatio: 1.5, // Ajuste del tamaño de los iconos
            children: [
              ClimateDataCard(
                icon: const FaIcon(FontAwesomeIcons.thermometerHalf, size: 20),
                label: 'Temperatura',
                value: '30°C',
              ),
              ClimateDataCard(
                icon: const FaIcon(FontAwesomeIcons.tint, size: 20),
                label: 'Humedad',
                value: '65%',
              ),
              ClimateDataCard(
                icon: const FaIcon(FontAwesomeIcons.solidLightbulb, size: 20),
                label: 'Intensidad Luz',
                value: '800 lux',
              ),
              ClimateDataCard(
                icon: const FaIcon(FontAwesomeIcons.cloudRain, size: 20),
                label: 'Precipitación',
                value: '12mm',
              ),
              ClimateDataCard(
                icon: const FaIcon(FontAwesomeIcons.wind, size: 20),
                label: 'Air Quality',
                value: 'AQI 50',
              ),
              ClimateDataCard(
                icon: const FaIcon(FontAwesomeIcons.fire, size: 20),
                label: 'Gases Tóxicos',
                value: '5ppm',
              ),
              ClimateDataCard(
                icon: const FaIcon(FontAwesomeIcons.sun, size: 20),
                label: 'Luz UV',
                value: '7',
              ),
              ClimateDataCard(
                icon: const FaIcon(FontAwesomeIcons.weightHanging, size: 20),
                label: 'Presión Atmosférica',
                value: '1015 hPa',
              ),
            ],
          );
        },
      ),
    );
  }
}

class ClimateDataCard extends StatelessWidget {
  final FaIcon icon;
  final String label;
  final String value;

  ClimateDataCard({
    required this.icon,
    required this.label,
    required this.value,
  });

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        icon,
        const SizedBox(height: 8), // Espacio entre el ícono y el texto
        Text(label),
        const SizedBox(height: 5),
        Text(
          value,
          style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
        ),
      ],
    );
  }
}
