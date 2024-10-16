import 'package:flutter/material.dart';
import 'edit_profile_screen.dart';
import 'login_screen.dart';

class ProfileScreen extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Center(
      // Usamos Center para centrar todo el contenido
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          mainAxisAlignment:
              MainAxisAlignment.center, // Centramos verticalmente
          children: [
            const CircleAvatar(
              radius: 50,
              backgroundImage: NetworkImage(
                  'https://via.placeholder.com/150'), // Imagen simulada
            ),
            const SizedBox(height: 20),
            const Text(
              'Usuario Simulado',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 10),
            const Text('correo@ejemplo.com'),
            const SizedBox(height: 20),
            ElevatedButton(
              onPressed: () {
                // Acción de editar perfil
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (context) => EditProfileScreen()),
                );
              },
              child: const Text('Editar Perfil'),
            ),
            const SizedBox(height: 20), // Espacio entre los botones
            ElevatedButton(
              onPressed: () {
                // Cerrar sesión y volver a la pantalla de inicio de sesión
                Navigator.pushReplacement(
                  context,
                  MaterialPageRoute(builder: (context) => LoginScreen()),
                );
              },
              style: ElevatedButton.styleFrom(
                backgroundColor:
                    Colors.red, // Color para el botón de cerrar sesión
              ),
              child: const Text('Cerrar Sesión'),
            ),
          ],
        ),
      ),
    );
  }
}
