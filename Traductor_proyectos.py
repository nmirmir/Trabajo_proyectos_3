def cargar_posiciones(nombre_fichero):
    posiciones = {}
    with open(nombre_fichero, 'r') as file:
        for linea in file:
            partes = linea.strip().split('\t')
            if len(partes) < 7 or partes[0].isdigit() is False:
                continue
            idx = int(partes[0])
            nombre = partes[-1]
            posiciones[nombre] = idx
    return posiciones

def cargar_plan(nombre_fichero):
    with open(nombre_fichero, 'r') as file:
        lineas = file.readlines()
    acciones = [linea.strip() for linea in lineas if linea.strip().startswith('(')]
    return acciones

def nombre_posicion(pilar, nivel, abierto=True):
    estado = 'A' if abierto else 'C'
    return f'P{pilar[1]}_P{nivel}_{estado}'

def posicion_ataque(pilar, abierto=True):
    estado = 'A' if abierto else 'C'
    return f'P{pilar[1]}_AT_{estado}'

def generar_movimiento(posiciones, nombre):
    return f"MOVER {posiciones[nombre]} 1000"

# Secuencia de posiciones para coger
def generar_secuencia_ir_y_coger(posiciones, pilar, nivel):
    return [
        generar_movimiento(posiciones, posicion_ataque(pilar, True)),
        generar_movimiento(posiciones, nombre_posicion(pilar, nivel, True)),
        generar_movimiento(posiciones, nombre_posicion(pilar, nivel, False)),
        generar_movimiento(posiciones, posicion_ataque(pilar, False)),
    ]

# Secuencia de posiciones para dejar
def generar_secuencia_ir_y_dejar(posiciones, pilar, nivel):
    return [
        generar_movimiento(posiciones, posicion_ataque(pilar, False)),
        generar_movimiento(posiciones, nombre_posicion(pilar, nivel, False)),
        generar_movimiento(posiciones, nombre_posicion(pilar, nivel, True)),
        generar_movimiento(posiciones, posicion_ataque(pilar, True)),
    ]

# Función de depuración para imprimir el estado actual de los pilares
def imprimir_estado_pilares(pilares):
    print("Estado actual de los pilares:")
    for pilar, discos in pilares.items():
        print(f"{pilar}: {discos}")

def procesar_plan(acciones, posiciones):
    salida = [generar_movimiento(posiciones, 'INICIO_ALT')]
    
    # Inicializar los pilares y los discos
    pilares = { 'p1': [], 'p2': [], 'p3': [] }
    discos = ['a', 'b', 'c', 'd']  # Del más pequeño al más grande
    
    # Estado inicial de los pilares
    pilares['p1'] = ['d', 'c', 'b', 'a']  # pilar p1 tiene los 4 discos (d en base, a arriba)
    pilares['p2'] = []
    pilares['p3'] = []
    
    # Imprimir el estado inicial de los pilares
    print("Antes")
    imprimir_estado_pilares(pilares)
    
    # Procesar cada acción
    for accion in acciones:
        print(f"\nProcesando acción: {accion}")
        
        # Acción: Cojer un disco de un pilar
        if accion.startswith("(cojer_de_pila"):
            _, disco, debajo, pilar = accion.strip("()").split()
            pila_actual = pilares[pilar]
            nivel = len(pila_actual) - pila_actual.index(disco)
            salida += generar_secuencia_ir_y_coger(posiciones, pilar, nivel)
            print(f'Actualizado Hay nivel {nivel} en el pilar {pila_actual}')
            pila_actual.remove(disco)
        
        # Acción: Dejar un disco en la mesa
        elif accion.startswith("(dejar_en_mesa"):
            _, disco, pilar = accion.strip("()").split()
            pila_actual = pilares[pilar]
            pila_actual.insert(0, disco)  # Colocar el disco en el nivel más bajo
            nivel = len(pila_actual) - pila_actual.index(disco)
            print(f'Actualizado Hay nivel {nivel} en el pilar {pila_actual}')
            salida += generar_secuencia_ir_y_dejar(posiciones, pilar, nivel)
        
        # Acción: Cojer un disco de la mesa
        elif accion.startswith("(cojer_de_mesa"):
            _, disco1, _, pilar = accion.strip("()").split()
            pila_actual = pilares[pilar]
            nivel = len(pila_actual) - pila_actual.index(disco1)  # Calcular el nivel al coger el disco
            salida += generar_secuencia_ir_y_coger(posiciones, pilar, nivel)
            print(f'Actualizado Hay nivel {nivel} en el pilar {pila_actual}')
            pila_actual.remove(disco1)
        
        # Acción: Dejar un disco en un pilar
        elif accion.startswith("(dejar_en_pila"):
            _, disco, debajo, pilar = accion.strip("()").split()
            pila_actual = pilares[pilar]
            
            # Colocar el disco debajo del disco indicado (si existe)
            if debajo in pila_actual:
                pila_actual.insert(pila_actual.index(debajo) + 1, disco)
            else:
                pila_actual.append(disco)  # Si no hay ningún disco debajo, lo ponemos encima

            # Calcular el nivel correcto basado en la longitud de la pila
            print(f"LEN ACTUAL DE LA PILA SIN CAMBAIR NIVEL = {len(pila_actual)}")
            print(f"El tamaño fijo de la pila + 1: {len(discos) + 1}")
            nivel = -len(pila_actual) + (len(discos) + 1) # El nivel será la longitud de la pila (1-indexed)
            print(f'Actualizado Hay nivel {nivel} en el pilar {pila_actual}')
            salida += generar_secuencia_ir_y_dejar(posiciones, pilar, nivel)
        
        # Imprimir el estado de los pilares después de la acción
        print("Despues:")
        imprimir_estado_pilares(pilares)
    
    return salida

def main():
    fichero_posiciones = 'posiciones.BR5G'
    fichero_plan = 'sas_plan.2'

    posiciones = cargar_posiciones(fichero_posiciones)
    acciones = cargar_plan(fichero_plan)
    movimientos = procesar_plan(acciones, posiciones)

    with open('ordenes_robot.BR5G', 'w') as out:
        with open(fichero_posiciones, 'r') as pos_file:
            contenido_posiciones = pos_file.read()
            out.write(contenido_posiciones)
            out.write('\n') #Salto random de linea
            #out.write('INICIO 1000\n') # Sinceramente no se si hay que poner aqui inicio o no 
        
        for movimiento in movimientos:
            out.write(movimiento + '\n')

    print("Archivo 'ordenes_robot.BR5G' generado correctamente.")

if __name__ == "__main__":
    main()

