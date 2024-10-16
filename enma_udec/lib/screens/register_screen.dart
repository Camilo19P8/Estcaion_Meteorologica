import 'package:flutter/material.dart';
import 'main_tab_screen.dart';

// Pantalla de registro
class RegisterScreen extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Registrarse'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Card(
          shape:
              RoundedRectangleBorder(borderRadius: BorderRadius.circular(10.0)),
          elevation: 5,
          child: Padding(
            padding: const EdgeInsets.all(16.0),
            child: Column(
              children: [
                const TextField(
                  decoration: InputDecoration(labelText: 'Nombre Completo'),
                ),
                const TextField(
                  decoration: InputDecoration(labelText: 'Correo Electrónico'),
                ),
                const TextField(
                  decoration: InputDecoration(labelText: 'Contraseña'),
                  obscureText: true,
                ),
                const SizedBox(height: 20),
                ElevatedButton(
                  onPressed: () {
                    Navigator.pushReplacement(
                      context,
                      MaterialPageRoute(builder: (context) => MainTabScreen()),
                    );
                  },
                  child: const Text('Registrarse'),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
