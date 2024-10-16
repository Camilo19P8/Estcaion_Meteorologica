import 'package:flutter/material.dart';
import 'package:enma_udec/screens/main_tab_screen.dart';
import 'package:enma_udec/screens/register_screen.dart';

class LoginScreen extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('ENMA'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Card(
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(10.0)),
              elevation: 5,
              child: Padding(
                padding: const EdgeInsets.all(16.0),
                child: Column(
                  children: [
                    const TextField(
                      decoration:
                          InputDecoration(labelText: 'Correo Electrónico'),
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
                          MaterialPageRoute(
                              builder: (context) => MainTabScreen()),
                        );
                      },
                      child: const Text('Iniciar Sesión'),
                    ),
                  ],
                ),
              ),
            ),
            TextButton(
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (context) => RegisterScreen()),
                );
              },
              child: const Text('Crear una cuenta'),
            ),
          ],
        ),
      ),
    );
  }
}
