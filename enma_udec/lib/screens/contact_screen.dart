import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';

class ContactScreen extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(16.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Text('Contáctanos',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
          const SizedBox(height: 20),
          const TextField(
            decoration: InputDecoration(labelText: 'Nombre'),
          ),
          const TextField(
            decoration: InputDecoration(labelText: 'Correo Electrónico'),
          ),
          const TextField(
            decoration: InputDecoration(labelText: 'Mensaje'),
            maxLines: 4,
          ),
          const SizedBox(height: 20),
          ElevatedButton(
            onPressed: () {
              // Acción de enviar formulario
            },
            child: const Text('Enviar'),
          ),
          const SizedBox(height: 40),
          const Text('Síguenos en nuestras redes sociales',
              style: TextStyle(fontSize: 18)),
          Row(
            children: [
              IconButton(
                icon: const FaIcon(FontAwesomeIcons.facebook),
                onPressed: () {},
              ),
              IconButton(
                icon: const FaIcon(FontAwesomeIcons.twitter),
                onPressed: () {},
              ),
              IconButton(
                icon: const FaIcon(FontAwesomeIcons.instagram),
                onPressed: () {},
              ),
            ],
          ),
        ],
      ),
    );
  }
}
