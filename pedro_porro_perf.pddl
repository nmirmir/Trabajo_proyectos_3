;; PROBLEM FILE - pedro_p.pddl
(define (problem pedro_p)
  (:domain pedro)

  (:objects
    l1 l2 l3 l4 l5 l6 - lata
    a1 a2 a3 a4 a5 a6 a7 a8 - almacen
  )

  (:init
    (pinza_libre)
    (lata_almacen l1 a1)
    (lata_almacen l2 a2)
    (lata_almacen l3 a3)
    (lata_almacen l4 a4)
    (lata_almacen l5 a5)
    (lata_almacen l6 a8)
  )

  (:goal (and
    (lata_almacen l1 a6)
    (lata_almacen l2 a6)
    (lata_almacen l3 a6)
    (lata_almacen l4 a7)
    (lata_almacen l5 a7)
    (lata_almacen l6 a7)
  ))
)