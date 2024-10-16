import 'package:flutter/material.dart';

class EditProfileScreen extends StatefulWidget {
  @override
  _EditProfileScreenState createState() => _EditProfileScreenState();
}

class _EditProfileScreenState extends State<EditProfileScreen> {
  bool isEditing = false;
  TextEditingController nameController =
      TextEditingController(text: 'Usuario Simulado');
  TextEditingController emailController =
      TextEditingController(text: 'correo@ejemplo.com');

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Editar Perfil'),
        actions: [
          IconButton(
            icon: Icon(isEditing ? Icons.save : Icons.edit),
            onPressed: () {
              setState(() {
                isEditing = !isEditing;
              });
              if (!isEditing) {
                // Guardar los cambios aquí
                ScaffoldMessenger.of(context).showSnackBar(
                  const SnackBar(
                    content: Text('Datos guardados exitosamente'),
                  ),
                );
              }
            },
          ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          children: [
            const CircleAvatar(
              radius: 50,
              backgroundImage: NetworkImage(
                  'https://via.placeholder.com/150'), // Imagen simulada
            ),
            const SizedBox(height: 20),
            TextField(
              controller: nameController,
              enabled: isEditing,
              decoration: const InputDecoration(labelText: 'Nombre'),
            ),
            const SizedBox(height: 10),
            TextField(
              controller: emailController,
              enabled: isEditing,
              decoration:
                  const InputDecoration(labelText: 'Correo Electrónico'),
            ),
            const SizedBox(height: 20),
            if (isEditing)
              ElevatedButton(
                onPressed: () {
                  setState(() {
                    isEditing = false;
                  });
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('Cambios guardados'),
                    ),
                  );
                },
                child: const Text('Guardar Cambios'),
              ),
          ],
        ),
      ),
    );
  }
}
