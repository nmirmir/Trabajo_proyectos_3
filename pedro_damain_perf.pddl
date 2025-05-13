;; DOMAIN FILE - pedro.pddl
(define (domain pedro)
  (:requirements :strips :typing)

  (:types
    lata - object
    almacen - object
  )

  (:predicates
    (lata_almacen ?l - lata ?a - almacen)
    (pinza_libre)
    (pinza_ocupada)
    (lata_cogida ?l - lata ?a - almacen)
    (lata_transportandose ?l - lata ?a1 - almacen ?a2 - almacen)
    (moviendose_a_almacen ?l - lata ?a - almacen)
    (en_posicion ?l - lata ?a - almacen)
    (lata_procesada ?l - lata)
  )

  ;; Preparar movimiento
  (:action a_mover_a_lata
    :parameters (?l - lata ?a - almacen)
    :precondition (and
      (pinza_libre)
      (not (pinza_ocupada))
      (lata_almacen ?l ?a)
      (not (moviendose_a_almacen ?l ?a))
      (not (lata_procesada ?l))
    )
    :effect (and
      (moviendose_a_almacen ?l ?a)
    )
  )

  ;; Coger lata
  (:action a_coger_lata
    :parameters (?l - lata ?a - almacen)
    :precondition (and
      (pinza_libre)
      (not (pinza_ocupada))
      (lata_almacen ?l ?a)
      (moviendose_a_almacen ?l ?a)
    )
    :effect (and
      (not (pinza_libre))
      (pinza_ocupada)
      (not (lata_almacen ?l ?a))
      (lata_cogida ?l ?a)
      (not (moviendose_a_almacen ?l ?a))
    )
  )

  ;; Mover lata entre almacenes
  (:action a_mover_lata
    :parameters (?l - lata ?a1 - almacen ?a2 - almacen)
    :precondition (and
      (pinza_ocupada)
      (not (pinza_libre))
      (lata_cogida ?l ?a1)
      (not (= ?a1 ?a2))
    )
    :effect (and
      (lata_transportandose ?l ?a1 ?a2)
    )
  )

  ;; Dejar lata
  (:action a_dejar_lata
    :parameters (?l - lata ?a1 - almacen ?a2 - almacen)
    :precondition (and
      (pinza_ocupada)
      (not (pinza_libre))
      (lata_cogida ?l ?a1)
      (lata_transportandose ?l ?a1 ?a2)
    )
    :effect (and
      (pinza_libre)
      (not (pinza_ocupada))
      (lata_almacen ?l ?a2)
      (en_posicion ?l ?a2)
      (lata_procesada ?l)
      (not (lata_cogida ?l ?a1))
      (not (lata_transportandose ?l ?a1 ?a2))
    )
  )
)
